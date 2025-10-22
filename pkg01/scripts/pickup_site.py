#!/usr/bin/env python3
import tkinter as tk

from numerical_keypad import NumericalKeypad

class pickupSite():
    def __init__(self):
        self.root = tk.Tk()
        self.keypad = NumericalKeypad(self.root, callback=self._useData)

    def _useData(self, calf_num):
        print("Received: ", calf_num)
        
        # TODO: serve il retain?
        self.clientMQTT.publish(topic=f'{self.topic}', payload=calf_num)

    def loop(self):
        self.root.bind('q', lambda e: self.root.quit())
        self.root.mainloop()


if __name__ == '__main__':
    pickup_site = pickupSite()
    pickup_site.loop()