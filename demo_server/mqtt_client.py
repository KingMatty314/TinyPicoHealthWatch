import random
import time
import pandas as pd
import threading, datetime, calendar
import paho.mqtt.client as mqtt

# Constants
accelx = []
accely = []
accelz = []

class mqttSub(mqtt.Client):    
    def on_connect(self, mqttc, obj, flags, rc):
        self.trial = 1
        self.recording_state = False
        print("On Connect: "+str(rc))

    def on_connect_fail(self, mqttc, obj):
        print("Connect failed")

    def on_message(self, mqttc, obj, msg):
        if str(msg.topic) == 'esp32watch/data/recording':
            if int(msg.payload) == 1 and not self.recording_state:
                self.recording_state = True
        
        if str(msg.topic) == 'esp32watch/data/recording':
            if int(msg.payload) == 0 and self.recording_state:
                self.recording_state = False
                self.trial += 1
                accel_dict = {'accelx' : accelx, 'accely' : accely, 'accelz' : accelz}
                df = pd.DataFrame(accel_dict)
                print("here")
                df.to_csv('trail' + str(self.trial) + '.csv')
                accelx.clear()
                accely.clear()
                accelz.clear()

        if str(msg.topic) == 'esp32watch/data/pedometer/accel_x' and self.recording_state:
            accelx.append(float(msg.payload))

        if str(msg.topic) == 'esp32watch/data/pedometer/accel_y' and self.recording_state:
            accely.append(float(msg.payload))

        if str(msg.topic) == 'esp32watch/data/pedometer/accel_z' and self.recording_state:
            accelz.append(float(msg.payload))

    def on_publish(self, mqttc, obj, mid):
        print("mid: "+str(mid))

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        print("Subscribed: " + str(mid) + " QOS: " + str(granted_qos))

    def on_log(self, mqttc, obj, level, string):
        print(string)

    def run(self):
        self.connect("127.0.0.1", 1883, 60)
        # Accelometer Sensor
        self.subscribe("esp32watch/data/pedometer/time", 0)
        self.subscribe("esp32watch/data/pedometer/accel_x", 0)
        self.subscribe("esp32watch/data/pedometer/accel_y", 0)
        self.subscribe("esp32watch/data/pedometer/accel_z", 0)
        # Oximeter Sensor
        self.subscribe("esp32watch/data/oximeter/time", 0)
        self.subscribe("esp32watch/data/oximeter/red", 0)
        self.subscribe("esp32watch/data/oximeter/ir", 0)
        # Device Status
        self.subscribe("esp32watch/data/battery", 0)
        self.subscribe("esp32watch/data/recording", 0)
        # Health Data
        self.subscribe("esp32watch/health/heartbeat", 0)
        self.subscribe("esp32watch/health/sp02", 0)
        self.subscribe("esp32watch/health/steps", 0)
        self.subscribe("esp32watch/health/activity", 0)
        while True:
            self.loop_read()

 

class mqttPub(mqtt.Client):
    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_connect_fail(self, mqttc, obj):
        print("Connect failed")

    def on_publish(self, mqttc, obj, mid):
        print("message published, id: " + str(mid))

    def run(self):
        self.connect("127.0.0.1", 1883, 60)
        while True:
            t = datetime.datetime.now()
            hour = t.hour
            min = t.minute
            sec = t.second
            dayOfWeek = calendar.day_name[t.weekday()]
            month = calendar.month_name[t.month]
            day = t.day
            year = t.year 
            self.publish("esp32watch/data/time/hr", str(hour))
            self.publish("esp32watch/data/time/min", str(min))
            self.publish("esp32watch/data/time/sec", str(sec))
            self.publish("esp32watch/data/time/day-of-week", dayOfWeek)
            self.publish("esp32watch/data/time/month", month)
            self.publish("esp32watch/data/time/day", str(day))
            self.publish("esp32watch/data/time/year", str(year))
            time.sleep(100)

# tesing web page and mqtt system with articial values
class testWebPageActivityData(mqtt.Client):
    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_connect_fail(self, mqttc, obj):
        print("Connect failed")

    def on_publish(self, mqttc, obj, mid):
        print("message published, id: " + str(mid))

    def run(self):
        self.connect("127.0.0.1", 1883, 60)
        steps = 0
        heartbeat = 80
        sp02 = 99
        activity = "walking"
        battery = 3.7
        while True:
            self.publish("esp32watch/health/heartbeat", heartbeat)
            self.publish("esp32watch/health/sp02", sp02)
            self.publish("esp32watch/health/steps", steps)
            self.publish("esp32watch/health/activity", activity)
            self.publish("esp32watch/data/battery", battery)
            battery = 3.7 + random.randint(-1, 1)
            steps += 1
            heartbeat = 80 + random.randint(-10, 20)
            sp02 = 99 + random.randint(-1, 1)
            if random.randint(0, 1) == 0:
                activity = "walking"
            else:
                activity = "standing"
            time.sleep(2)

# tesing web page and mqtt system with articial values
class testWebPageRawData(mqtt.Client):
    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_connect_fail(self, mqttc, obj):
        print("Connect failed")

    def on_publish(self, mqttc, obj, mid):
        print("message published, id: " + str(mid))

    def run(self):
        self.connect("127.0.0.1", 1883, 60)
        red = 1200
        ir = 2100
        accel = 1.0
        while True:            
            self.publish("esp32watch/data/pedometer/accel", accel)
            self.publish("esp32watch/data/oximeter/red", red)
            self.publish("esp32watch/data/oximeter/ir", ir)
            accel = 1.0 + round(random.uniform(-0.3, 0.3), 3)
            red = 1200 + random.randint(-100, 100)
            ir = 2100 + random.randint(-100, 100)
            time.sleep(0.1)



if __name__ == "__main__":
    #sub_client = mqttSub()
    #pub_client = mqttPub()
    #sub=threading.Thread(target=sub_client.run)
    #pub=threading.Thread(target=pub_client.run)
    #sub.start()
    #pub.start()

    # for testing
    test_client_activity = testWebPageActivityData()
    test_client_data = testWebPageRawData()
    test_activity = threading.Thread(target=test_client_activity.run)
    test_data = threading.Thread(target=test_client_data.run)
    test_activity.start()
    test_data.start()
