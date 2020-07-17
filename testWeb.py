import remi.gui as gui
from remi import start, App

class MyApp(App):
    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def setVals(self, emg, temp, hr, spo2, x, y, z):
        self.emg = emg
        self.temp = temp
        self.hr = hr
        self.spo2 = spo2
        self.x = x
        self.y = y
        self.z = z

    def main(self):
        self.emg = ""
        self.temp = ""
        self.x = ""
        self.y = ""
        self.z = ""
        self.hr = ""
        self.spo2 = ""

        container = gui.VBox(width=500, height=200)
        self.lbl = gui.Label('Hello world!')
        self.emgLabel = gui.Label("EMG: " + self.emg)
        self.tempLabel = gui.Label("Temp: " + self.temp + " F")
        self.hrLabel = gui.Label("HR: " + self.hr)
        self.spo2Label = gui.Label("SpO2: " + self.spo2)
        self.xLabel = gui.Label("Accel-X: " + self.x)
        self.yLabel = gui.Label("Accel-Y: " + self.y)
        self.zLabel = gui.Label("Accel-Z: " + self.z)

        self.bt = gui.Button('Press me!')

        # setting the listener for the onclick event of the button
        self.bt.onclick.do(self.on_button_pressed)

        # appending a widget to another, the first argument is a string key
        container.append(self.lbl)
        container.append(self.emgLabel)
        container.append(self.tempLabel)
        container.append(self.hrLabel)
        container.append(self.spo2Label)
        container.append(self.xLabel)
        container.append(self.yLabel)
        container.append(self.zLabel)

        container.append(self.bt)

        # returning the root widget
        return container

    # listener function
    def on_button_pressed(self, widget):
        self.lbl.set_text('Button pressed!')
        self.bt.set_text('Hi!')

# starts the web server
#start(MyApp, address='192.168.1.11', port=8000, multiple_instance=False, enable_file_cache=True, update_interval=0.1, start_browser=True)
