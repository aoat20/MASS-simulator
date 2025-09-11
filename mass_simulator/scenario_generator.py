import numpy as np 
import dearpygui.dearpygui as dpg
import json

class ScenarioGenerator():
    def __init__(self):
        
        scen_dict = {}
        dpg.create_context()


        dpg.create_viewport(title="MASS Simulator",
                            width=1000,
                            height=1000)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        
    def _setup_window(self):

        pass

    def _set_up_map(self):
        pass

    def _add_new_boat(self):
        pass

    def _define_new_area(self):
        pass


