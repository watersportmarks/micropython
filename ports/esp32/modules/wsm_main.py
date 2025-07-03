from machine import Pin, I2C, PWM, DAC
import time
from bno08x_i2c import *
import ads1x15
import math
from ubinascii import unhexlify
import wsm
import os
from machine import WDT
import esp32
import bmm150

# Constants
GPS_I2C_ADDRESS = 0x42
IMU_GOOD_ACCURACY_THR = 5
IMU_SAVE_CALIBRATION_DELAY = 10 # loop 1Hz 250 # About 10 seconds (main loop @ about 25 Hz)
internet_available=0
db_access_status=0
MIN_WIDTH=990		# temporary. otherwise 1100-1900
MAX_WIDTH=2010

# global variables
heading = 0	 # from IMU task
headingFilt = 1000 # Init at 1000 (outside heading range) in order to recognize it need to be set with the sensor value at start
# IMUmode=0x02  # 1="geomag" ; 2="fusion"
timeGPS="000001"
lat=45.8 # gps latitude
lon=-8.8 # gps longitude
oldLat=lat; oldLon=lon
NumSat=0; FixQuality= 0; HorDilPos=1000;
#SNR=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
GPSprecision=99; GPSdeltaDist=0; GPSheading=0
#SNRsum=0
warnings=" "
PhoneNumber=""
GPSnormPrec=[99, 10, 1.5, 99, 0.1, 0.8, 99,99,99] # normal (best) GPS precision depending on fix quality
freshGPS=0; GPS_HZ=5; time_refreshGPS=0.2 # time between fresh data
date_day = 0
date_month = 0
date_year = 0
btn_free_pushed = False
print("starting main.py")

# LED init
strobo = PWM(Pin(4), freq=100, duty_u16=0) # 100 hz, 10ms; off
#strobo = Pin(4, Pin.OUT, value=0)
#strobo.on()
## time.sleep(1)
#strobo.off()

global_status_warnings = 0

nmea_rmc_disabled = False

# Buttons init
freeButton = Pin(33, Pin.IN, Pin.PULL_UP)
FIXButton = Pin(25, Pin.IN, Pin.PULL_UP)

# Motors init
PWMright = PWM(Pin(14), freq=50, duty_u16=4915) # 50 hz, 20ms; 1.5 ms
PWMright.duty_ns(1500_000)     # 1500us 
PWMleft = PWM(Pin(12), freq=50, duty_u16=4915) # 50 hz, 20ms; 1.5 ms
PWMleft.duty_ns(1500_000)     # 1500us 

# Audio init
#AUDIO_OUT = DAC(Pin(26))  # create an DAC object acting on a pin
#AUDIO_OUT.write(128)      # set a raw analog value in the range 0-255, 50% now
audio_en = Pin(27, Pin.OUT, value=1)
audio_en.off()
AUDIO_OUT = PWM(Pin(26), freq=5000, duty=512) # 50 hz, 20ms; 1.5 ms

# I2C init
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# ADC init
#adcMode=2 #0=use INA256, 1=adc old board, 2=adc pcb v1.0
RESISTOR_R1 = 100.0 #kohm
RESISTOR_R2 = 10.0
VOLTAGE_DIVIDER = (RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))
ADC_TO_AMP = (0.256/32768.0)*1000.0
AMP_TO_ADC = (32768.0/0.256)/1000.0
adc = None
try:
    adc = ads1x15.ADS1015(i2c, address=0x48, gain=1) # Range is +/- 4.096V.
except:
    adc = None
temp=20.0
volt=16.0
amp=0.1
wsm.set_amp(amp)
mAh=10000
wsm.set_mah(mAh)

# If there is a problem with the I2C bus then the following call blocks forever.
#i2cDev=i2c.scan() # returns 72 (0x48 ADS1015); 74 0x4A BNO085; 66 GPS
#print("I2C devices found:",i2cDev)

#i2c.readfrom(0x48, 1)
ADC_TO_VOLT = (4.096/(32768.0*VOLTAGE_DIVIDER))
if adc != None:
    try:
        volt = adc.read(4,0)
    except:
        volt = 0
else:
    volt = 0
volt = volt*ADC_TO_VOLT # conversione corretta? 
wsm.set_volt(volt)
print("volt",volt)

# IMU init
bno = None
try:
    bno = BNO08X_I2C(i2c, address=0x4A, debug=False)
    #bno = BNO08X_I2C(i2c, debug=False)
    bno.calibration() # calibrate accel + mag
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR) # default every 50 ms
    #print("IMU configured")
except:
    wsm.print_log("Cannot init IMU\n")
    bno = None

# GPS init
startTgps = time.ticks_ms()
bufmsb = unhexlify("fd")  # bytes in buffer,
data = unhexlify("ff")  # data stream
# m8n ublox disable GLL, GSA, VTG
# enable RMC (RMC will be disabled once date is parsed) and GSV (Satellites in View)
# GGA enabled by default
try:
    i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n', False)
    i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n', False)
    i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n', False)
    #i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,RMC,0,0,0,0,0,0*47\r\n', False)
    i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,RMC,1,1,1,1,0,0*47\r\n', False) # Enable RMC to get Date
    #i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,GSV,0,20,0,0,0,0*6B\r\n', False) # print each 20
    i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,GSV,0,0,0,0,0,0*59\r\n', False) # Disable GSV
    # See "UBX-CFG-RATE" from "u-blox M8 Receiver Description"
    # \xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A => set update rate to 5hz
    # \xB5\x62\x06\x08\x00\x00\x0E\x30 => get current rate
    #i2c.writeto(GPS_I2C_ADDRESS, b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A\xB5\x62\x06\x08\x00\x00\x0E\x30', False) # 5Hz + read setting
    i2c.writeto(GPS_I2C_ADDRESS, b'\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A', False) # 5Hz
    #i2c.writeto(GPS_I2C_ADDRESS, b'\xB5\x62\x06\x08\x06\x00\xF4\x01\x01\x00\x01\x00\x0B\x77', False) # 2Hz
except:
    print("Cannot init GPS")
    pass

# Magnetometer init
mag_degrees = 0
mag_offsets_max = [-1000, -1000, -1000]
mag_offsets_min = [1000, 1000, 1000]
mag_offsets = [0, 0, 0]
bmm = bmm150.BMM150(i2c, address=0x10)

def set2Range(angle):
    if angle > 180:
        angle = angle-360
    if angle <= -180:
        angle = angle+360		# set to range -180;180 deg
    return angle
def limitDuty(duty):   # set to range 1000;2000us
    if duty < MIN_WIDTH:
        duty = MIN_WIDTH
    if duty > MAX_WIDTH:
        duty = MAX_WIDTH
    return duty
def limitMotor(m,max):   # set to range -max,max
    if max<0:
        max=0
        warnings+=";warning function limitMotor had negative limnit"
    if m < -max:
        m = -max
    if m > max:
        m = max
    return m
def limitIncrement(val,old_val,inc):   # limit increment, no limit to decrement
    if (val>0) and ((val-old_val)>inc):
        val=old_val+inc
    if (val<0) and ((val-old_val)<-inc):
        val=old_val-inc
    return val
def limitIncDec(val,old_val,inc):   # limit increment and decrement
    if (val-old_val)>inc:
        val=old_val+inc
    if (val-old_val)<-inc:
        val=old_val-inc
    return val

def i2c_read_reg(dev, reg, length):
    i2c.writeto(dev, reg, False)
    return i2c.readfrom(dev, length)

def decode(coord):
    # Converts DDDMM.MMMM -> DDD.DDDDDD float
    x = coord.split(b'.')
    head = x[0]
    #tail =0
    #if len(x)>0:
    #	tail = x[1]
    try:
        deg = float(head[0:-2])
    except:
        deg=45
    try:
        min = float(coord)-deg*100
    except:
        min=0
    #print("c:%s , d:%3.1f, m:%2.4f" % (coord,deg, min))
    return (deg +min/60)

def read_gps():
    global oldLat, oldLon, startTgps, global_status_warnings, nmea_rmc_disabled, warnings
    global lat, lon, timeGPS, FixQuality, GPSdeltaDist
    global GPSheading, GPSprecision, freshGPS, GPS_HZ
    global date_day, date_month, date_year
    try:
        buflen = int.from_bytes(i2c_read_reg(GPS_I2C_ADDRESS, bufmsb, 2), "big")
        #print("buflen = " + str(buflen))        
        if buflen > 0:
            nmeas = i2c_read_reg(GPS_I2C_ADDRESS, data, buflen).decode().splitlines()
            if buflen>80:
                wsm.print_log("warning, gps buffer becoming full.." + str(buflen) + "\n")

            for nmea in nmeas:
                #print("nmea = " + str(nmea))
                
                if nmea_rmc_disabled == False:
                    if (nmea[0:6] == '$GNRMC'):
                        sdata = nmea.split(',')
                        date_string = sdata[9]
                        if date_string:  # Possible date stamp found
                            date_day = int(date_string[0:2])
                            date_month = int(date_string[2:4])
                            date_year = int(date_string[4:6])                        
                            nmea_rmc_disabled = True
                            i2c.writeto(GPS_I2C_ADDRESS, b'$PUBX,40,RMC,0,0,0,0,0,0*47\r\n', False)
                            print("date: " + str((date_day,date_month,date_year)))
                            warnings += ";date:"+str((date_day,date_month,date_year))
                        
                nmea = bytes(nmea, 'utf-8')            
                if (nmea[0:6] == b'$GPGGA') or (nmea[0:6] == b'$GNGGA'):
                    #print(" ", data)
                    sdata = nmea.split(b',')
                    #print ("c %f" % float(sdata[1]))
                    try:
                        timeGPS = sdata[1] #sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
                        lat = decode(sdata[2]) #latitude
                        dirLat = sdata[3][0]     #latitude direction N/S
                        if dirLat == 83: #b'S':
                            lat=-lat
                        lon = decode(sdata[4]) #longitude
                        dirLon = sdata[5][0]      #longitude direction E/W
                        #print("dirLon = " + str(dirLon) + ", " + str(sdata[5][0]))
                        if dirLon == 69: #b'E':
                            lon=-lon
                        FixQuality= float(sdata[6])
                        NumSat= float(sdata[7])
                        HorDilPos= float(sdata[8])
                        height= float(sdata[9])
                    except Exception as e:
                        #print("gga err: " + str(sdata) + "," + str(e))
                        #timeGPS="000002"
                        lat=45.8
                        dirLat ='N'
                        lon=-8.8
                        dirLon ='E'
                        FixQuality= 0
                        NumSat= 0
                        HorDilPos= 200
                        height= 100
                    wsm.set_gps_fix_quality(FixQuality)
                    wsm.set_lat(lat)
                    wsm.set_lon(lon)
                    wsm.set_gpstime(timeGPS)
                    moved_lat = (lat - oldLat) * 111319.4 # last second movement in meter
                    moved_lon =  111319.4 * (lon -oldLon) * math.cos(math.radians(lat)) # in meter
                    GPSdeltaDist = math.sqrt(moved_lat*moved_lat + moved_lon*moved_lon) # distance from last fix
                    wsm.set_gps_delta_dist(GPSdeltaDist)
                    GPSheading = math.degrees(math.atan2(moved_lon, moved_lat))  # angle of the last second movement
                    wsm.set_gps_heading(GPSheading)
                    oldLat=lat
                    oldLon=lon
                    delta_lat = (lat - wsm.get_start_lat()) * 111319.4 # from start in meter
                    wsm.set_delta_lat(delta_lat)
                    delta_lon =  40075000 * (lon - wsm.get_start_lon()) /360 * math.cos(math.radians(lat)) # in meter
                    wsm.set_delta_lon(delta_lon)
                    GPSprecision=GPSnormPrec[int(FixQuality)] * HorDilPos  # simplified estimation of precision in m                    
                    if GPSprecision>999:
                        GPSprecision=999
                    wsm.set_gps_precision(GPSprecision)
                    time_refreshGPS = time.ticks_ms() - startTgps
                    time_refreshGPS = time_refreshGPS/1000
                    #print("time_refreshGPS = " + str(time_refreshGPS))
                    if time_refreshGPS>2:
                        time_refreshGPS = 2 # in case time is too long to avoid crazy effects
                    if time_refreshGPS<0.01:
                        time_refreshGPS = 0.01 #  to avoid crazy effects   
                    if time_refreshGPS > 0.8: # 5Hz was not accepted. try again
                        wsm.print_log("refresh GPS slow:" + str(time_refreshGPS) + "\n")
                    GPS_HZ=1/time_refreshGPS
                    #print("GPS_HZ = " + str(GPS_HZ))
                    startTgps = time.ticks_ms()
                    freshGPS=1
                    wsm.set_fresh_gps(True)
                    if FixQuality > 1: # FixQuality: 0=no fix, 1=minimal, 2=assisted, 3=differential
                        global_status_warnings &= ~(0x04)
                    else:
                        global_status_warnings |= (0x04)                    

    except UnicodeError as e: # if stray \xff chars make it into the buffer, don't crash
        print("UnicodeError: " + str(e))
        pass
    except IndexError as e: # if data is not transferred properly, don't crash
        print("IndexError: " + str(e))
        pass
    except Exception as e:
        print("readgps err: " + str(e))
        pass

def read_compass():
    global mag_degrees, btn_free_pushed
    try:
        magx, magy, magz, _ = bmm.measurements
        #print(f"x: {magx}uT, y: {magy}uT, z:{magz}uT")

        if freeButton.value()==0: # FREE BUTTON PUSHED
            btn_free_pushed = True
            if(mag_offsets_max[0] < magx):
                mag_offsets_max[0] = magx
            if(mag_offsets_max[1] < magy):
                mag_offsets_max[1] = magy
            if(mag_offsets_max[2] < magz):
                mag_offsets_max[2] = magz
            if(mag_offsets_min[0] > magx):
                mag_offsets_min[0] = magx
            if(mag_offsets_min[1] > magy):
                mag_offsets_min[1] = magy
            if(mag_offsets_min[2] > magz):
                mag_offsets_min[2] = magz  
            mag_offsets[0] = (mag_offsets_max[0] + mag_offsets_min[0])/2
            mag_offsets[1] = (mag_offsets_max[1] + mag_offsets_min[1])/2
            mag_offsets[2] = (mag_offsets_max[2] + mag_offsets_min[2])/2
            #print("mag offsets: " + str(mag_offsets))
        else:
            if btn_free_pushed:
                btn_free_pushed = False
                wsm.print_log("mag offsets: " + str(mag_offsets))

        compass = math.atan2(magx-mag_offsets[0], magy-mag_offsets[1])
        # Convert to range 0..360
        #if compass < 0:
        #    compass += 2 * math.pi
        #if compass > 2 * math.pi:
        #    compass -= 2 * math.pi    
        mag_degrees = compass * 180 / math.pi
        mag_degrees = set2Range(mag_degrees+90);
        wsm.set_mag_degrees(mag_degrees)
        #print("mag heading:  %.2f "%mag_degrees)
    except Exception as e:
        print("error reading magnetometer " + str(e))

def start_control_loop():
    global freshGPS, GPSdeltaDist, GPS_HZ, GPSheading, GPSprecision, time_refreshGPS, warnings, amp, volt, adc, mAh, headingFilt, global_status_warnings, VOLTAGE_DIVIDER, MIN_WIDTH, MAX_WIDTH, timeGPS, date_day, date_month, bno, mag_degrees, lat, lon

    MOTlimit = 500
    SOFT_ACC_STEP = 2 # When goal changed, for 10 seconds is active

    max_time = 0
    min_time = 1000000
    imu_best_confidence = 100
    imu_save_cal_count = 0
    imu_cal_done_count = 0
    amp_offset_raw = 0
    base_consumption_amp = 0.1
    first_measure = 1
    btnPushed = False
    btnCounter = 0
    #stroboToggle = 0
    log_count = 0 # counter for log and prints, each 1s
    delta_time = [0] * 25
    heading = 0
    mR = 0
    mL = 0
    warn = ""
    distance = 0
    controlType = 0
    desFW = 0
    yawStart = 0
    goalChanged = 0
    k_headingDrift = 0.10 # 0.15   slower filter 26apr2024
    headingDriftFiltered = 0.0
    headingDrift = 0.0
    driftVariation = 0.0
    HeadingWarning = 0
    magCal = 0
    forwardControl = 0
    alpha = 0
    delta_a = 0
    old_a = 0
    integr_a = 0.0
    rot = 0
    firstPosFix = 0
    #limitAmp = 12 # from here limit the current 12A ~190W
    limitAmp = 25 # from here limit the current 18A ~290W
    mR_duty = 1500
    mL_duty = 1500
    vWind = -1  # estimated wind speed, resolution 0.1m/s
    angWind = -1 # estimated wind direction [0-360]
    AtPos = 0 # at position =1, or driving to a position =0
    goalChangedCounter = 0
    AlertToSend = 0
    rotProblCounterR = 0
    rotProblCounterL = 0
    stallProtectionLeft = 0
    stallProtectionRight = 0
    stallLeftCounter = 0
    stallRightCounter = 0
    ctrlChanged = 0
    vel = 0
    old_vel = 0
    old_rot = 0
    controlT = 0.040	# time [s] period of the control cycle 0.05-> 20Hz
    disableControlAndStop = 0 # If the distance to the goal is too big then disable control and stop the mark
    audioErrorCounter = 0
    firstDistError = 1
    global_status_errors = 0
    last_mR_duty = 1500
    last_mL_duty = 1500
    updateCntrOnDB = 0 # they are not needed...
    updateGoalOnDB = 1 # they are not needed...
    # position control PID parameters
    ka = 1.0  #1.5 k too much strong
    kaD = 1.5  # k for diff rotation
    kaI = 0.2  # small Integral to force rotation
    kWind = 1 # coefficient dependent on type of motors
    integr_d = 0 # integrated (low pass) dist for PID	
    kd = 20 # k for distance (was 40 on big)
    kdD = 20 # k for diff distance
    kdI = 9 # k for integr distance    
    delta_d = 0 # dist-old - dist for PID
    delta_d2 = 0
    old_d = 0
    old_d2 = 0
    SPEEDlimit = 0.6 # m/s then start to decrease the motor speed
    tempMOTlimit = MOTlimit  # eventually decreased the tempMOTlimit if GPS speed too high or electrical current too high
    long_message_count = 0
    longMessage = ""
    delta_lat = 0
    delta_lon = 0
    xx = 0
    yy = 0
    x_des = 0
    y_des = 0
    boaID = 0
    motorType = 2
    IMUupsidedown_back = 0 # electronic board normal position VS upsidedown and facing backward
    forceForward = 0
    imuPitchRoll = 1
    roll = 0
    pitch = 0
    confidence = 0
    confidence_prev = 0
    fastFlash = 1
    led_phase = 0
    ledPWM = 1
    conf_shunt_low = 0
    freshGPS = 0 # this variable is defined to read from BT if connected, but then it is not used because handled from C side (set_fresh_gps/get_fresh_gps). It shuuld be deleted after modfiyng the "bt_updated" function.
    bnoErrorCount = 0
    useGpsSim7600 = False
    wsm.use_gps_sim7600(useGpsSim7600)
    useMagBmm150 = False

    boaID = wsm.get_mark_id()
    print("boa id = " + str(boaID))
    wsm.print_log("boa id = " + str(boaID) + "\r\n")

    # Change PID control parameters, ...
    if boaID == 220100304702492: # 5B
        IMUupsidedown_back = 1

    if boaID == 220100304698904: # 5D
        IMUupsidedown_back = 1

    if boaID == 220100304698944: # 5F 
        conf_shunt_low = 1

    if boaID == 220100304698868: # 6A
        conf_shunt_low = 1

    if boaID == 220100304703464: # 6B
        IMUupsidedown_back = 1

    wsm.set_force_forward(forceForward)

    if motorType==2:
        #ka=1
        #kaD=1.5
        #kaI=0.2  # small Integral to force rotation
        #integr_a=0.0
        #kd=20 # k for distance (was 40 on big)
        #kdD=20 # k for diff distance
        #kdI=9 # k for integr distance
        #kWind=1 # coefficient dependent on type of motors
        kWind=0.5
    if motorType==3:# fast medium cina
        ka=0.8
        kWind=2.0
        kaD=0.3  # k for diff rotation
        kd=30 # 20 # k for distance
        kdD=30# 20 # k for diff distance
        kdI=0 # 6 # k for integr distance
        MIN_WIDTH=1300		# temporary. otherwise 1100-1900
        MAX_WIDTH=1700
        MOTlimit=110
    if motorType==4:# big 48v cina
        ka=0.8
        kaD=0.3  # k for diff rotation
        kd=20 # k for distance
        kdD=20 # 10 # k for diff distance
        kdI=0 # k for integr distance
        kWind=1.0
    if motorType==5:  # slower vd70
        ka=1.2
        kWind=2.0
        kaD=0.3  # k for diff rotation
        kd=40# 30 # k for distance
        kdD=30 # 20 # k for diff distance
    if motorType==6:# big 48v cina e per traino
        ka=3
        kaD=4  # k for diff rotation
        kd=40 # k for distance
        kdD=100 # k for diff distance
        kdI=0 # k for integr distance
        kWind=1.0
    if motorType==7:# big 48v cina e per traino + ESC big water
        ka=1
        kaD=1  # k for diff rotation
        kd=30 # k for distance
        kdD=50 # k for diff distance
        kdI=0 # k for integr distance
        kWind=1.0

    wdt = WDT(timeout=10000)  # enable it with a timeout of 10s

    start = time.ticks_ms()  

    wsm.print_log("xGPS; yGPS; heading;(CalMag);  mR, mL, GPSprecision, lat, lon, ,GPStime hhmmss, heading drift, mAh, Volt, Amp, Mag\n")

    while 1:
        try:
            wdt.feed()
            # main loop at 25 Hz
            delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference
            #if max_time < delta:
            #    max_time = delta
            #if min_time > delta:
            #    min_time = delta
            #print("time="+str(delta))
            delta_time[log_count] = delta
            if delta < 40:
                time.sleep_ms(40-delta)  # period 40ms: 25Hz
            start = time.ticks_ms()
            
            
            if fastFlash>0:
                ## LED fast flash, turn off as soon as possible
                strobo.duty_u16(0) # PWM off
            
            if btnPushed:
                btnCounter = btnCounter + 1
                if btnCounter == 10:
                    #print("btn audio stop")
                    audio_en.off()
                    btnPushed = False
            else:
                controlType = wsm.get_control_type()

            if wsm.bt_updated():
                desFW, yawStart, controlType, start_lat, start_lon, goalChanged, xx, yy, delta_lat, delta_lon, freshGPS, x_des, y_des, k_headingDrift = wsm.get_bt_update()
                #print("desFW = " + str(desFW))
            #******* read imu at 25Hz
            #print("bno.euler = " + str(bno.euler))
            #start = time.ticks_ms()
            if bno != None:
                try: # sometimes I get error here...
                    pitch, roll, P, confidence = bno.euler  # pitch and roll inverted to be aligned as in the mark
                    #print("roll = " + str(int(roll)))
                    #print("pitch = " + str(int(pitch)))
                    bnoErrorCount = 0
                except Exception as e:
                    print("imu read error: " + str(e))
                    bnoErrorCount = bnoErrorCount + 1
                    if bnoErrorCount == 3:
                        bnoErrorCount = 0
                        #print("reinit bno...")
                        wsm.print_log("reinit bno...")
                        bno = BNO08X_I2C(i2c, address=0x4A, debug=False)
                        bno.calibration() # calibrate accel + mag
                        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR) # default every 50 ms
            else:
                roll = 0
                pitch = 0
                P = 0
                confidence = 0
            #delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference
            #delta_time[log_count] = delta
            if(imuPitchRoll == 0):
                roll = 0
                pitch = 0
            else:
                roll = int(roll)
                wsm.set_pitch(pitch)
                pitch = int(pitch)
                if IMUupsidedown_back == 1:
                    pitch = set2Range(pitch - 180.0)
            heading=int(P)
            read_compass()
            if useMagBmm150:
                heading=int(mag_degrees)
            wsm.set_heading(heading)
            if headingFilt == 1000: # At boot headingFilt is initialized at 1000 in order to start with the same value read from the sensor
                headingFilt = heading
            else:
                headingFilt = int(headingFilt*0.9 + heading*0.1)
            wsm.set_heading_filt(headingFilt)
            
            confidence = int(confidence*100) 
            magCal=0
            if confidence < 3:
                magCal = 3
                #if confidence != confidence_prev:
                #    wsm.print_log("Confidence OK:" + str(magCal) + "\n")
            elif confidence < 10:
                magCal = 2
            elif confidence < 40:
                magCal = 1
                #if confidence != confidence_prev:
                #    wsm.print_log("Low confidence:" + str(magCal) + "\n")
            #else: #>=40
                #if confidence != confidence_prev:
                #    wsm.print_log("Low confidence:" + str(magCal) + "\n")
            wsm.set_mag_cal(magCal)
            confidence_prev = confidence

        #     option to save IMU calibration below in tasks at 1HZ

            #******* read GPS (when data ready) at 5Hz
            # When data available:
            # - with micropyGPS library read takes about 33-35 ms
            # - with manual parsing of only GGA read takes 8-10 ms
            # 1 every 5 loops the data are available.
            # message length (nmea GGA) is 75 bytes.
            #start_gps = time.ticks_ms()
            if useGpsSim7600:
                freshGPS = wsm.get_fresh_gps_sim7600()
                wsm.set_fresh_gps_sim7600(False)
                lat = wsm.get_lat_sim7600()
                lon = wsm.get_lon_sim7600()
                GPSdeltaDist = wsm.get_gps_delta_dist_sim7600()
                GPSheading = wsm.get_gps_heading_sim7600()
                GPSprecision = wsm.get_gps_precision_sim7600()
                time_refreshGPS = 0.2
                GPS_HZ = 5
            else:
                read_gps()
            #delta_gps = time.ticks_diff(time.ticks_ms(), start_gps) # compute time difference
            #delta_time[log_count] = delta_gps
            #print("gps time = " + str(delta_gps))
            if freshGPS==1:
                #headingDrift filter decay
                if headingDriftFiltered>0:
                    #headingDriftFiltered-=(0.15 * time_refreshGPS)
                    headingDriftFiltered-=(1.5 * time_refreshGPS)
                    if headingDriftFiltered<0:	# passed from + to - as security set to 0
                        headingDriftFiltered=0
                else:
                    headingDriftFiltered+=(1.5 * time_refreshGPS)
                    if headingDriftFiltered>0:
                        headingDriftFiltered=0
                if (controlType==1 or controlType==3) and forwardControl==1 and (GPSdeltaDist*GPS_HZ)>0.5 and math.fabs(mR-mL)<100:
                    headingDrift = set2Range(GPSheading - heading)
                    driftVariation= set2Range(headingDrift-headingDriftFiltered) # to avoid bug when passing 180'
                    #headingDriftFiltered= headingDriftFiltered + (0.3/GPS_HZ)*driftVariation # add only part of the variation
                    headingDriftFiltered= headingDriftFiltered + (k_headingDrift*time_refreshGPS)*driftVariation # add only part of the variation
                    headingDriftFiltered=set2Range(headingDriftFiltered)
                    if math.fabs(headingDrift)>55 :
                        if controlType==3:
                            HeadingWarning+=1
                        warn=";w%d, drift:%d" %(HeadingWarning,headingDrift)
                        warnings+=warn
                        # !!! condition still not sure if moving back because of wind/waves or wrong IMU heading!!!
                        # Warning level 4, time to recalibrate the IMU? or use the GPS angle? or use the heading+drift?
                        if HeadingWarning>3:
                            #relativeYaw = set2Range(heading + headingDriftFiltered)	## the IMU heading has drifted, add the estimated drift
                            HeadingWarning=4 # limit increment of warning
                    if math.fabs(headingDrift)<35 :
                        HeadingWarning-=1
                        if HeadingWarning<0:
                            HeadingWarning=0
            ### attention, if distance >4m, we add ALWAYS the heading drift calculated from GPS. good idea??
            relativeYaw=heading
            if distance>4:
                relativeYaw = set2Range(heading + headingDriftFiltered)	## the IMU heading has some drift, add the estimated drift

            #******** CONTROL yawStart=heading
            if controlType ==1:   # yaw stabilization at desired yaw
                alpha=set2Range(yawStart-heading )
                delta_a=alpha-old_a
                old_a=alpha
                integr_a=limitMotor(0.95*integr_a+0.05*alpha,100)  # low pass angle as I term @20Hz
                rot=ka*alpha + kaD*delta_a + kaI*integr_a # PID controller
                rot=limitMotor(rot,400)# temporary limit 400, later 500
                mR= desFW + rot
                mL= desFW + -rot
                if firstPosFix==0: # before first fix do not control angle. used to check motors
                    mR= desFW
                    mL= desFW
                # limit current (amp) to limitAmp
                if amp > limitAmp:
                    decr=limitAmp/amp
                    mR= mR * decr
                    mL= mL * decr
                    warn=";%dA" %(int(amp))
                    warnings+=warn
                # limitation from pitch
                if (pitch>22 and pitch<67) : # 
                    mR=mR *(67-pitch)/45
                    mL=mL *(67-pitch)/45
                if (pitch>-90 and pitch<-45): # 
                    mR=mR *(pitch+90)/45
                    mL=mL *(pitch+90)/45
                if (pitch>66 or pitch<-89):
                    mR=0
                    mL=0                
                mR_duty=1500 + mR
                wsm.set_mr_duty(int(mR_duty))
                mL_duty=1500 + mL
                wsm.set_ml_duty(int(mL_duty))
                vWind=-1
                wsm.set_vwind(vWind)
                angWind=-1
                wsm.set_ang_wind(angWind)

            if freshGPS==1:
                SPEEDlimit = wsm.get_speed_limit()
                #print("SPEEDlimit = " + str(SPEEDlimit))
                freshGPS=0
                if(useGpsSim7600):
                    delta_lat = wsm.get_delta_lat_sim7600()
                    delta_lon = wsm.get_delta_lon_sim7600()
                else:
                    delta_lat = wsm.get_delta_lat()
                    delta_lon = wsm.get_delta_lon()
                xx = delta_lat
                yy = delta_lon
                # x_des and y_des on servo_demo.py are always zero...
                distance= math.sqrt((x_des-xx)*(x_des-xx) + (y_des-yy)*(y_des-yy)) # calculate only with new GPS pos
                delta_d=distance-old_d	# delta distance calculation only with new GPS pos
                delta_d2=distance-old_d2
                old_d2=old_d
                old_d=distance
                if controlType ==3: # limit speed filter only if GPS controlled
                    if ((GPSdeltaDist*GPS_HZ) > SPEEDlimit) and (delta_d<0): # gps distance * GPSrate = speed
                        #print("tempMOTlimit = " + str(tempMOTlimit))
                        tempMOTlimit-=5   # if faster then decrease the motor limit
                        if tempMOTlimit<50:
                            tempMOTlimit=50
                    else:
                        tempMOTlimit+=6   # otherwise increase the motor limit
                        if tempMOTlimit > MOTlimit:
                            tempMOTlimit=MOTlimit  # untill the defined max
                    if distance <2:
                        AtPos=1
                        SPEEDlimit=0.6
                        wsm.set_speed_limit(SPEEDlimit)
                    if goalChanged==1:
                        warnings+="; goal changed "+str(int(distance))+"m"
                        AtPos=0
                        goalChanged=0
                        goalChangedCounter = 0
                    if distance > 7: #no valid wind estimation
                        vWind=-1
                        wsm.set_vwind(vWind)
                        angWind=-1
                        wsm.set_ang_wind(angWind)
                    if AtPos==1 and distance >7 and goalChanged==0:
                        AlertToSend |= (0x01)
                        wsm.set_alert(AlertToSend)
                        AtPos=0
                        warnings+="; alert 6m"
                    ## alert for abnormal rotation reaction, maybe 1 motor blocked or damaged
                    if rot>90 and delta_a<5 :
                        rotProblCounterR+=1
                        if rotProblCounterR>30:
                            rotProblCounterR=0
                            warnings+="motR~?" + str(int(rot)) + "; " + str(int(delta_a))
                            AlertToSend |= (0x02)
                            wsm.set_alert(AlertToSend)
                            stallProtectionRight = 1
                            stallRightCounter = 0
                    else:
                        rotProblCounterR-=1
                        if rotProblCounterR<0:
                            rotProblCounterR=0
                    if rot<-90 and delta_a>-5 :
                        rotProblCounterL+=1
                        if rotProblCounterL>30:
                            rotProblCounterL=0
                            warnings+="motL~?" + str(int(rot)) + "; " + str(int(delta_a))
                            AlertToSend |= (0x04)
                            wsm.set_alert(AlertToSend)
                            stallProtectionLeft = 1
                            stallLeftCounter = 0
                    else:
                        rotProblCounterL-=1
                        if rotProblCounterL<0:
                            rotProblCounterL=0

                    if ctrlChanged==1:
                        warnings+="; ctrl changed"
                        ctrlChanged=0
                        
                    # vWind= int(math.sqrt((delta_lat)*(delta_lat) + (delta_lon)*(delta_lon)) *30.0) # wind estimation proportional to distance to goal
                    # estimated wind speed, resolution 0.1m/s
                    vWind= int(((mR_duty-1500)+(mL_duty-1500))/3*	kWind) # wind estimation proportional to motor speed
                    wsm.set_vwind(vWind)
                    dirGoal= int( math.degrees(math.atan2(-delta_lon,-delta_lat))) # estimated wind direction... simply direction to goal.
                    if dirGoal <0:
                        dirGoal+=360
                    angWind= 0.95*angWind+0.05*dirGoal  # simple low pass filter. requires propper reset and filter adjustment
                    wsm.set_ang_wind(int(angWind))

            if controlType ==3:   # GPS position control
                # limit current (amp) to limitAmp
                if amp > limitAmp:
                    tempMOTlimit-= 4		# rapidly decrease max mot speed, 37 @1s
                    if tempMOTlimit<15: # try to keep at least some motor speed
                            tempMOTlimit=15	
                    # if current not high anymore, above in the GPS code will indrease again 3 @1s
                    #decr=limitAmp/amp
                    #mR= mR * decr
                    #mL= mL * decr
                    warn=";%dA" %(int(amp))
                    warnings+=warn

                firstPosFix=1
                # position control (Astolfi+eurisitc+PID)
                integr_d=limitMotor(0.8*integr_d+0.2*distance,10)  # low pass distance as I term. and limit to 10
                alpha= set2Range(math.degrees(math.atan2(y_des-yy, x_des-xx))- relativeYaw) # angle to goal
                integr_a=limitMotor(0.95*integr_a+0.05*alpha,100)  # low pass angle as I term @20Hz
                delta_a=alpha-old_a
                old_a=alpha
                #if ((alpha>-100) and(alpha<100))or(distance>1.5):	# move forward
                if (forceForward==1) or ((alpha>-110) and(alpha<110))or(distance>1.2):	# move forward
                    forwardControl=1
                    if ((alpha<-50) or (alpha>50)):	# direction not good
                        if integr_d>2:
                            integr_d=2 # limit integral part
                    #vel=kd*distance + kdD*delta_d + kdI*integr_d - 2*math.fabs(alpha)
                    distance2=distance
                    if distance>20:
                        distance2=20
                    vel=kd*distance2 + kdD*delta_d+ kdD*delta_d2 + kdI*integr_d - 1*math.fabs(alpha)
                    if math.fabs(alpha) >80: # first rotate then advance
                        vel=0
                    if vel<0:
                        vel=0
                else:
                    forwardControl=0
                    integr_d=0
                    alpha=set2Range(alpha-180)
                    #vel=kd*distance + kdD*delta_d + kdI*integr_d - 2*math.fabs(alpha)
                    vel=kd*distance + kdD*delta_d - 1*math.fabs(alpha)
                    if vel<0:
                        vel=0
                    vel= -vel # move backward
                vel=limitMotor(vel,tempMOTlimit)
                # slow accelerations
                vel= limitIncrement(vel, old_vel, 40*controlT) # 2@20Hz, 40 @1Hz
                if goalChangedCounter < 25*10: # active for about 10 seconds (control run @ 25 hz) after goal is changed from DB
                    goalChangedCounter = goalChangedCounter + 1
                    if (vel - old_vel) > SOFT_ACC_STEP: # Too big acceleration, avoid it to avoid consumption's peaks...in about 10 seconds it goes to max
                        vel = old_vel + SOFT_ACC_STEP
                    if (vel - old_vel) < -SOFT_ACC_STEP: # Too big deceleration, avoid it to avoid consumption's peaks...in about 10 seconds it goes to max
                        vel = old_vel - SOFT_ACC_STEP
                    if vel > 200:
                        vel = 200
                    if vel < -200:
                        vel = -200
                mR= vel
                mL= vel
                old_vel=vel
                
                rot=ka*alpha + kaD*delta_a + kaI*integr_a # PID controller
                rot=limitMotor(rot,MOTlimit)
                if distance<1:
                    rot=rot/2  # slower if close
                if forceForward==1: # Boa di traino...
                    if distance<3:
                        #rot=rot/2  # slower if close
                        mR= vel/2
                        mL= vel/2

                # rot= limitIncrement(rot, old_rot, 40*controlT) # better change rotation fast or slow??
                old_rot=rot
                mR += rot
                mL += -rot
                # if not too close, avoid turning on spot with negative motor speed
                if forwardControl==1 and distance>0.6 and vel<math.fabs(rot) and math.fabs(alpha)<70:
                    if mR<0:
                        mL=mL-mR # add instead both speeds to 1 motor while the other at 0
                        mR=0
                    if mL<0:
                        mR=mR-mL
                        mL=0
                # 9mar larger 2x circle
                if distance < (2.0*GPSprecision*1.5): #(GPSprecision*1.5): # slower reaction zone 
                    coef=distance/(2.0*GPSprecision*1.5) #coef=distance/(GPSprecision*1.5)
                    mR=mR*coef
                    mL=mL*coef
                    # proportional vs 3 circles
                
                if 	magCal < 0: # security
                    mR=0
                    mL=0
                if 	mR<-60 : # avoid motor out of water
                    mR=-60
                if 	mL<-60 : # avoid motor out of water
                    mL=-60
                                    
                #if 	distance>50 : # security
                #	mR=0
                #	mL=0
                
                if distance > 2000:
                    disableControlAndStop = 1
                    if firstDistError == 1:
                        firstDistError = 0
                        warnings+="; too big dist "+str(int(distance))+"m"
                        audioErrorCounter = 199
                    mR = 0
                    mL = 0
                else:
                    disableControlAndStop = 0
                    firstDistError = 1

                # limitation from pitch
                if (pitch>22 and pitch<67) : # 
                    mR=mR *(67-pitch)/45
                    mL=mL *(67-pitch)/45
                if (pitch>-90 and pitch<-45): # 
                    mR=mR *(pitch+90)/45
                    mL=mL *(pitch+90)/45
                if (pitch>66 or pitch<-89):
                    mR=0
                    mL=0
                    
                mR_duty=1500 + mR
                wsm.set_mr_duty(int(mR_duty))
                mL_duty=1500 + mL
                wsm.set_ml_duty(int(mL_duty))

                if (global_status_errors == 1) or (disableControlAndStop == 1):
                    audioErrorCounter = audioErrorCounter + 1;
                    if audioErrorCounter == 200: # play a sound every 10 seconds
                        audioErrorCounter = 0	
                        #audio_play_once("/home/pi/Desktop/boa/wav/status_error.wav")
        #         end control type 3


        #******** CONTROL STOP
            if controlType ==0:   # turn off motors
                mR=0
                mL=0
                old_vel=0
                old_rot=0
                mR_duty=1500
                mL_duty=1500
                wsm.set_mr_duty(int(mR_duty))
                wsm.set_ml_duty(int(mL_duty))
                vWind=-1
                wsm.set_vwind(vWind)
                angWind=-1
                wsm.set_ang_wind(angWind)
                headingDriftFiltered=0
                
            # fast LED flahes
            if fastFlash==1 and controlType==3 and distance<3:   # flash mode IN POSITION
                ## LED sequence: 1, 0, 0, 0, 1, 20x0  @55ms
                if led_phase==0 or led_phase==4:
                    strobo.duty_u16(ledPWM*256) # RPi code pwm was between 0..255, now the range is 0..65535
                else:
                    strobo.duty_u16(0) # PWM off
                led_phase+=1
                if led_phase>30:
                    led_phase=0			
            # fast LED flahes
            if fastFlash==1 and controlType==3 and distance>=3:   # flash mode IN MOVEMENT
                ## LED sequence: 1, 0, 0, 0, 1, 20x0  @55ms
                if led_phase==0 or led_phase==4 or led_phase==8:
                    strobo.duty_u16(ledPWM*256) # RPi code pwm was between 0..255, now the range is 0..65535
                else:
                    strobo.duty_u16(0) # PWM off
                led_phase+=1
                if led_phase>15:
                    led_phase=0			
            # fast LED flahes
            if fastFlash==1 and controlType!=3:   # flash mode IN STANDBY
                ## LED sequence: 1, 0, 0, 0, 1, 20x0  @55ms
                if led_phase==0:# or led_phase==4:
                    strobo.duty_u16(ledPWM*256) # RPi code pwm was between 0..255, now the range is 0..65535
                else:
                    strobo.duty_u16(0) # PWM off
                led_phase+=1
                if led_phase>30:
                    led_phase=0	        


        # #SERVO HANDLING
            if stallProtectionRight == 1:
                warnings += ";stallProtectionRight"
                PWMright.duty_ns(1500_000)     # 1500us stop
                stallRightCounter += 1
                if(stallRightCounter == 10):
                    stallProtectionRight = 0
            else:
                mR_duty=int(limitDuty(mR_duty))
                wsm.set_mr_duty(int(mR_duty))
                #mR_duty1= limitIncDec(mR_duty, last_mR_duty, 5) # 5@25Hz, 125 @1Hz
                mR_duty1= limitIncDec(mR_duty, last_mR_duty, 10) # 10@25Hz, 250 @1Hz
                if mR_duty1<1480:
                    PWMright.duty_ns((mR_duty1-20)*1000)
                elif mR_duty1>1525:
                    PWMright.duty_ns((mR_duty1+15)*1000)
                else: #if mR_duty==1500:
                    PWMright.duty_ns(1500_000)
                last_mR_duty=mR_duty1
                
            if stallProtectionLeft == 1:
                warnings += ";stallProtectionLeft"
                PWMleft.duty_ns(1500_000)     # 1500us stop
                stallLeftCounter += 1
                if(stallLeftCounter == 10):
                    stallProtectionLeft = 0
            else:
                mL_duty=int(limitDuty(mL_duty))
                wsm.set_ml_duty(int(mL_duty))
                #mL_duty1= limitIncDec(mL_duty, last_mL_duty, 5) # 5@25Hz, 125 @1Hz
                mL_duty1= limitIncDec(mL_duty, last_mL_duty, 10) # 10@25Hz, 250 @1Hz
                if mL_duty1<1485:
                    PWMleft.duty_ns((mL_duty1-20)*1000)
                elif mL_duty1>1525:
                    PWMleft.duty_ns((mL_duty1+15)*1000)
                else:#if mL_duty==1500:		
                    PWMleft.duty_ns(1500_000)     # 1500us stop
                last_mL_duty=mL_duty1
        
        #### ****** TASKS at 1Hz
            log_count = log_count + 1
            if log_count == 25:# each 25 times: 1Hz
                log_count = 0

                #stroboToggle = 1 - stroboToggle
                #if stroboToggle:
                #    strobo.on()
                #else:
                #    strobo.off()
                
                if adc != None:
                    try:
                        adc.gain = 1 # 1x 4.096V
                        volt = adc.read(4,0)
                    except:
                        volt = 0
                else:
                    volt = 0    
                volt = volt*ADC_TO_VOLT # conversione corretta? 
                wsm.set_volt(volt)
                #print(volt)
                volt2=volt+0.07*amp # 0.04*amp
                if volt2<14.5 and volt!=0:
                    global_status_warnings |= (0x01)
                    # send a SMS
                    AlertToSend |= (0x08)  # bit4
                    wsm.set_alert(AlertToSend)
                else:
                    global_status_warnings &= ~(0x01)
                if volt2<12.2 and volt!=0: # was 13.2V  3V3 per cell
                    wsm.print_log("standby because of low voltage\n")
                    # send a SMS
                    AlertToSend |= (0x08)  # bit4
                    wsm.set_alert(AlertToSend)
                    controlType =0   # turn off motors to save batteries
                    wsm.set_control_type(controlType) 
                    updateCntrOnDB=1
                    global_status_errors |= (0x04)
                else:
                    global_status_errors &= ~(0x04)
                if volt2<11.0 and volt!=0: # was 12.0  3V0 per cell
                    # send a SMS
                    AlertToSend |= (0x08)
                    wsm.set_alert(AlertToSend)
                    controlType =0   # should shutdown completely
                    updateCntrOnDB=1
                    wsm.set_control_type(controlType)            
                    wsm.print_log("shutdown\n")
                    # time.sleep(2)
                    # check_call(['sudo', 'poweroff'])

                if adc != None:
                    try:
                        adc.gain = 5 # 16x (0.256V)
                        amp = adc.read(4,1)
                        if conf_shunt_low == 1:
                            amp = amp*5                                     
                    except:
                        amp = 0
                else:
                    amp = 0
                #print("curr = " + str(amp))
                if first_measure == 1:
                    first_measure = 0
                    wsm.print_log("I measure[V] = " + str(amp) + "\n")
                    curr_amp = amp*ADC_TO_AMP
                    wsm.print_log("curr_amp = " + str(curr_amp) + "\n")
                    #if curr_amp > base_consumption_amp:
                    diff_amp = curr_amp - base_consumption_amp
                    wsm.print_log("diff_amp = " + str(diff_amp) + "\n")
                    amp_offset_raw = diff_amp*AMP_TO_ADC
                    wsm.print_log("amp_offset_raw = " + str(amp_offset_raw) + "\n")
                    #else:
                    #	amp_offset_raw = 0
                #print("amp raw = " + str(amp))
                amp -= amp_offset_raw
                #print("amp raw - offset = " + str(amp))
                if(amp < 0):
                    amp = 0
                amp = amp*ADC_TO_AMP
                mAh -= amp/3.6
                wsm.set_mah(mAh)

                #for val in delta_time:
                #    print(str(val)+",", end="")
                
                #print("max="+str(max_time)+",min="+str(min_time))
                #min_time = 1000000
                #max_time = 0
                
                # Buttons
                if FIXButton.value()==0: # FIX BUTTON PUSHED
                    print("fixPushed")
                    btnPushed = True
                    btnCounter = 0
                    audio_en.on()
                    #print("button:GPSfix")
                # FIX button -> fix here
                    #audio_play_once("/home/pi/Desktop/boa/wav/button_feedback.wav")
                    controlType=3
                    wsm.set_control_type(controlType)
                    xx=0			# reset x y 
                    yy=0
                    wsm.set_start_lat(lat)
                    wsm.set_start_lon(lon)
                    x_des=0
                    y_des=0
                    goalChanged=1
                    updateGoalOnDB=1
                    updateCntrOnDB=1
                    wsm.set_update_goal_on_db(updateGoalOnDB);
                    warnings+="; button FIX"
                    wsm.set_delta_lat(0)
                    wsm.set_delta_lon(0)
                    wsm.set_delta_lat_sim7600(0)
                    wsm.set_delta_lon_sim7600(0)                    
                if freeButton.value()==0: # FREE BUTTON PUSHED
                    print("freePushed")
                    btnPushed = True
                    btnCounter = 0
                    audio_en.on()
                    if controlType == 0: # Give audio status when "free" is pressed twice or more times.
                        if global_status_errors == 0:
                            if global_status_warnings == 0: # Status ok
                                #audio_play_once("/home/pi/Desktop/boa/wav/status_ok.wav")
                                warnings+="; status ok"
                            else: # Status warning
                                #audio_play_once("/home/pi/Desktop/boa/wav/status_warning.wav")
                                warnings+="; status warning" #print("status warning")
                        else: # Status error
                            #audio_play_once("/home/pi/Desktop/boa/wav/status_error.wav")
                            warnings+="; status error" #print("status error")
                    #else:
                        #audio_play_once("/home/pi/Desktop/boa/wav/button_feedback.wav")
                    controlType=0
                    wsm.set_control_type(controlType)
                    updateCntrOnDB=1
                    warnings+="; button FREE"

                # LED strong flash or sequence
                if fastFlash==0:
                    ## LED sequence: 0, 0, 1, GPSfix=2, magCal>1 
                    if led_phase==0:
                        strobo.duty_u16(0) # PWM off
                    if led_phase==1:
                        strobo.duty_u16(0) # PWM off
                    if led_phase==2:
                        strobo.duty_u16(65535) # PWM on
                    if (led_phase==3 and FixQuality<2):
                        strobo.duty_u16(0) # PWM off
                    if led_phase==4:
                        strobo.duty_u16(0) # PWM off
                        if magCal>1:
                            strobo.duty_u16(65535) # PWM on
                    led_phase+=1
                    if led_phase>4:
                        led_phase=0

                if isinstance(timeGPS, bytes):
                    #print("timeGPS is bytes")
                    timeGPSstr = timeGPS.decode('utf-8').split('.')[0]
                else:
                    #print("timeGPS is string")
                    timeGPSstr = timeGPS.split('.')[0]
                if timeGPSstr == "":
                    timeGPSstr = "000001"
                #print("timeGPStemp = " + timeGPSstr + ", len=" + str(len(timeGPSstr)) + ", isdigit=" + str(timeGPSstr.isdigit()))
                #localTime=int(timeGPS[0])*10+int(timeGPS[1])+int(timeGPS[2])*0.16+int(timeGPS[2])*0.166+int(timeGPS[2])*0.0166 # time in h decimal => old method from servo_demo.py
                if len(timeGPSstr)==6 and timeGPSstr.isdigit():
                    localTime = int(timeGPSstr[0:2]) + int(timeGPSstr[2:4])*0.0166						
                else:
                    localTime = 0
                #print("h decimal = " + str(localTime))            
                localTime+=lon/15 # UTC time + longitude delta
                if localTime>24:
                    localTime-=24
                if localTime<0:
                    localTime+=24
                day=date_month*30+date_day   # day of the year
                sunset=4+3*math.sin(day/365.0*3.14)			# very simple estimation of sunset or ~daylight
                if localTime>=(12-sunset) and localTime<=(12+sunset):
                    ledPWM=255		# max
                if localTime<=(12-sunset-3) or localTime>=(12+sunset+3):
                    ledPWM=5			# min./sto	
                if localTime>(12+sunset) and localTime<(12+sunset+3):
                    ledPWM=int(255-(250/3)*(localTime-(12+sunset))	)	# linear
                if localTime>(12-sunset-3) and localTime<(12-sunset):
                    ledPWM=int(5+(250/3)*(localTime-(12-sunset-3)))		# linear
                #print(day, sunset, localTime , ledPWM, timeGPS)                    

                #print("timeGPS =  " + str(timeGPS))
                #print("timeGPSstr = " + timeGPSstr)
                #print("%d" % (int(float(timeGPSstr))))

                # SEND MESSAGE
                long_message_count = long_message_count + 1
                if(long_message_count == 10): # every 10 seconds
                    long_message_count = 0
                    longMessage="%1.2f;%1.2f;%3.0f;(%d);%1.0f;%1.0f;%.1f;%.7f;%.7f;%d;%.0f;%.1f;%.2f;%.1f;%3.0f" % (xx,yy,heading, magCal,mR_duty-1500,mL_duty-1500, GPSprecision, lat, lon,int(float(timeGPSstr)),headingDriftFiltered,mAh,volt,amp,mag_degrees)
                else:
                    longMessage="%1.2f;%1.2f;%3.0f;%1.0f;%1.0f;%.0f;%.1f;%3.0f" % (xx,yy,heading, mR_duty-1500,mL_duty-1500,headingDriftFiltered,amp,mag_degrees)	
                #confidence
                #print(longMessage+warnings)
                
                #start = time.ticks_ms()
                #log_file.write(longMessage+warnings)
                #log_file.write("\r\n")
                #log_file.flush()
                wsm.print_log(longMessage+warnings +"\r\n")
                #delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference
                #print("log write time = " + str(delta))
                
                warnings=""
                
                #print(delta_lat, delta_lon, heading, "[", magCal, lat, lon)

                # check if it is time and quality to save a IMU calibration
                if bno != None:
                    try:
                        if not bno.calibration_completed():
                            imu_cal_done_count = imu_cal_done_count + 1
                            if imu_cal_done_count == 5:#125: # If after 4 seconds calibration still not started, then resend command
                                imu_cal_done_count = 0
                                bno.calibration() # calibrate accel + mag
                                print("restart imu cal")
                        if confidence < IMU_GOOD_ACCURACY_THR:
                            if confidence < imu_best_confidence:
                                imu_save_cal_count = imu_save_cal_count + 1
                                if imu_save_cal_count == IMU_SAVE_CALIBRATION_DELAY:
                                    imu_save_cal_count = 0
                                    imu_best_confidence = confidence
                                    bno.save_calibration_data()
                                    print("save calibration")
                            else:
                                imu_save_cal_count = 0    
                    except:
                        print("imu calib error")

        # end tasks at 1 Hz
        except Exception as e: # while loop general exception
            print("while loop error " + str(e))
            wsm.print_log("while loop error " + str(e))

         


