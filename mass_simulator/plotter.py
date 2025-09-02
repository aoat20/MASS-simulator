import dearpygui.dearpygui as dpg
from mass_simulator.agent import Agent, OtherVessel
import os
import numpy as np
from mass_simulator.general import *


class Plotter():
    def __init__(self,
                 vessels,
                 xy_lims,
                 control=True):

        self.play = True
        self.playspeed = 10
        self.t_n = 0
        self._change_waypoints = False
        self._waypoints_temp = {}
        self._send_waypoints = False
        self._vessel_id_foc = vessels["agent"].vessel_id
        vessel_N = len(vessels)

        dpg.create_context()
        dpg.configure_app(init_file=os.path.join(os.path.dirname(__file__),
                                                 'dpg.ini'),
                          docking=True,
                          docking_space=True)

        # with dpg.window(label="Master Controls"):
        #     with dpg.group(horizontal=True):
        #         dpg.add_button(label="Save Window pos",
        #                        callback=self.save_init)

        self._initialise_map(xy_lims)

        with dpg.item_handler_registry(tag="click_handler"):
            dpg.add_item_clicked_handler(callback=self._mouse_callback,
                                         button=dpg.mvMouseButton_Left)
        dpg.bind_item_handler_registry(item="map_plot_tag",
                                       handler_registry="click_handler")

        self._add_vessels(vessels=vessels)
        if control:
            self._initialise_controls()
        self._initialise_variable_viewer(vessel_N)

        dpg.create_viewport(title="MASS Simulator",
                            width=600,
                            height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def _get_plot_colors(self, n):
        cols = [[0.8660, 0.3290, 0],
                [0.3290, 0.7130, 1.0000],
                [0.0660, 0.4430, 0.7450],
                [0.9960, 0.5640, 0.2620],
                [0.4540, 0.9210, 0.8540],
                [0, 0.6390, 0.6390]]
        return [c*255 for c in cols[n]]

    def _setup_plot_themes(self, n):
        col = self._get_plot_colors(n)

        with dpg.theme(tag=f'line_theme_{n}'):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
                                    dpg.mvPlotMarker_Cross,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize,
                                    6,
                                    category=dpg.mvThemeCat_Plots)
        with dpg.theme(tag=f'new_wp_theme_{n}'):
            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
        return col

    def _initialise_map(self,
                        xy_lims):
        with dpg.window(label="World",
                        tag="world_window",
                        no_close=True):
            dpg.add_plot(label="World",
                         height=-1,
                         width=-1,
                         equal_aspects=True,
                         delay_search=True,
                         tag="map_plot_tag")
            dpg.add_plot_axis(dpg.mvXAxis,
                              parent="map_plot_tag",
                              pan_stretch=False,
                              tag="map_x_axis")
            dpg.add_plot_axis(dpg.mvYAxis,
                              parent="map_plot_tag",
                              pan_stretch=False,
                              tag="map_y_axis")
            dpg.bind_item_handler_registry(f"map_y_axis",
                                           "click_handler")

    def _initialise_controls(self):
        with dpg.window(label='Controls',
                        tag="tag_control"):
            dpg.add_checkbox(label="Play",
                             callback=self._set_play,
                             default_value=True,
                             tag='tag_play')
            dpg.add_drag_float(label="Playspeed",
                               min_value=0.1,
                               max_value=100,
                               default_value=self.playspeed,
                               callback=self.set_playspeed)
            dpg.add_checkbox(label="Change warpoints",
                             callback=self._change_waypoints_check_cb)
            dpg.add_button(label="Clear waypoints",
                           callback=self._clear_wps)

    def add_time_scrubber(self, t_max):
        with dpg.window(label='Time'):
            dpg.add_slider_float(label="Time, min",
                                 min_value=1,
                                 max_value=t_max-1,
                                 clamped=True,
                                 callback=self._set_time,
                                 tag="time_slider",
                                 width=1000)

    def _set_time(self, sender, app_data):
        self.t_n = app_data

    def get_time(self):
        t_rnd = round(self.t_n)
        return t_rnd

    def set_time(self, t):
        self.t_n = t
        dpg.set_value("time_slider",
                      t)

    def _set_play(self, sender, app_data):
        self.play = app_data

    def set_play(self, play):
        self.play = play
        dpg.set_value('tag_play',
                      play)

    def _clear_wps(self, sender, app_data):
        self._waypoints_temp = {}
        for b in dpg.get_aliases():
            if "waypoint_temp_" in b:
                dpg.set_value(b,
                              [[]])

    def _change_waypoints_check_cb(self, sender, app_data):
        self._change_waypoints = app_data
        if app_data:
            self._waypoints_temp = {}
            self._send_waypoints = False
        else:
            self._send_waypoints = True

    def get_waypoint_updates(self):
        if self._send_waypoints and self._waypoints_temp != {}:
            wp_ret = self._waypoints_temp
            self._waypoints_temp = {}
            self._clear_wps([], [])
            return wp_ret
        else:
            return []

    def _initialise_variable_viewer(self,
                                    vessels_N):
        with dpg.window(label="AIS Data",
                        no_close=True):
            dpg.add_text(default_value=f"Time: -",
                         tag="time_tag")
            with dpg.table(tag='other_vessels_table',
                           header_row=True,
                           row_background=True,
                           borders_innerH=True,
                           borders_innerV=True,
                           borders_outerH=True,
                           borders_outerV=True):
                dpg.add_table_column()
                for n in range(vessels_N-1):
                    dpg.add_table_column(label="-",
                                         tag=f"tag_id_v{n}")
                with dpg.table_row():
                    dpg.add_text('CPA, yds')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_cpa_v{n}")
                with dpg.table_row():
                    dpg.add_text('TCPA')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_tcpa_v{n}")
                with dpg.table_row():
                    dpg.add_text('Range, yds')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_range_v{n}")
                with dpg.table_row():
                    dpg.add_text('Bearing, deg')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_bearing_v{n}")

    def _add_vessels(self, vessels):
        v: Agent
        n = 0
        for v_key in vessels:
            # Setup plot colours for the boat
            col = self._setup_plot_themes(n)

            # Set up line for history
            dpg.add_line_series(label=v_key,
                                x=[],
                                y=[],
                                parent='map_y_axis',
                                tag=f"tag_hist_{v_key}")

            # Set up triangle for vessel locations
            with dpg.draw_node(parent="map_plot_tag",
                               tag=f"tag_draw_{v_key}"):
                dpg.draw_triangle(p1=[-50, -50],
                                  p2=[0, 150],
                                  p3=[50, -50],
                                  color=col,
                                  fill=col,
                                  thickness=0.1,
                                  tag=f"tag_triangle_{v_key}")

            # Add annotation for vessel information
            dpg.add_plot_annotation(label=f"{v_key}\n"
                                    "speed -kn \n"
                                    "course -deg",
                                    default_value=[0, 0],
                                    offset=[5, 5],
                                    parent='map_plot_tag',
                                    tag=f"annot_{v_key}")

            # Add crosses for waypoints
            dpg.add_scatter_series(label=f"{v_key} waypoints",
                                   x=[],
                                   y=[],
                                   tag=f"waypoint_plot_{v_key}",
                                   parent="map_y_axis")
            dpg.add_line_series(x=[],
                                y=[],
                                parent="map_y_axis",
                                tag=f"waypoint_line_{v_key}",
                                segments=True)

            dpg.add_scatter_series(label=f"{v_key} new waypoints",
                                   x=[],
                                   y=[],
                                   tag=f"waypoint_temp_{v_key}",
                                   parent="map_y_axis")

            # Change the colours of the plots
            dpg.bind_item_theme(f"tag_hist_{v_key}",
                                f'line_theme_{n}')
            dpg.bind_item_theme(f"waypoint_plot_{v_key}",
                                f'line_theme_{n}')
            dpg.bind_item_theme(f"waypoint_line_{v_key}",
                                f'line_theme_{n}')
            dpg.bind_item_theme(f"waypoint_temp_{v_key}",
                                f'new_wp_theme_{n}')

            n += 1

    def _select_boat(self,
                     xy,
                     vessel_id: str = ""):
        if vessel_id:
            self._vessel_id_foc = vessel_id
            return

        # Check if it's near to a boat and switch to that boat if so
        for b in dpg.get_aliases():
            if "tag_hist_" in b:
                x = dpg.get_value(b)[0][-1]
                y = dpg.get_value(b)[1][-1]
                d = compute_distance(xy, [x, y])
                if d < 200:
                    self._vessel_id_foc = b.replace('tag_hist_', '')
                    return True

    def _add_temporary_waypoints(self, xy):
        # otherwise, update the waypoints of the current focussed vessel
        if self._change_waypoints:
            if self._vessel_id_foc not in self._waypoints_temp:
                self._waypoints_temp[self._vessel_id_foc] = []
            self._waypoints_temp[self._vessel_id_foc].append(xy)
            dpg.set_value(f"waypoint_temp_{self._vessel_id_foc}",
                          list(zip(*self._waypoints_temp[self._vessel_id_foc])))

    def _mouse_callback(self, sender, app_data):
        mouse_pos = dpg.get_plot_mouse_pos()

        if self._select_boat(mouse_pos):
            return

        self._add_temporary_waypoints(mouse_pos)

    def update_vessels(self,
                       vessels: dict):
        v: Agent
        for v in vessels.values():
            self._update_vessel_plot(v.vessel_id,
                                     v.xy,
                                     v.xy_hist,
                                     v.course_deg,
                                     v.speed_kn,
                                     v.waypoints)
            if v.vessel_id == self._vessel_id_foc:
                self._update_vessels_table(v.other_vessels)

    def _update_vessels_table(self,
                              vessels: dict):
        v: OtherVessel
        n = 0
        for key, v in vessels.items():
            col = dpg.get_item_configuration(f"tag_triangle_{key}")['color']
            dpg.configure_item(f"tag_id_v{n}",
                               label=key)
            dpg.highlight_table_column('other_vessels_table',
                                       n+1,
                                       [c*255 for c in col])
            dpg.set_value(f"tag_cpa_v{n}", f"{v.cpa_yds:.0f}")
            dpg.set_value(f"tag_tcpa_v{n}",
                          f"{np.floor(v.tcpa_s/60):.0f}min {v.tcpa_s%60:.0f}s")
            dpg.set_value(f"tag_range_v{n}", f"{v.range_yds:.0f}")
            dpg.set_value(f"tag_bearing_v{n}", f"{v.bearing_deg:.1f}")
            n += 1

    def _update_vessel_plot(self,
                            vessel_id,
                            xy,
                            xy_hist,
                            course_deg,
                            speed_kn,
                            waypoints):
        course_rad = np.deg2rad(course_deg)
        dpg.apply_transform(f"tag_draw_{vessel_id}",
                            transform=dpg.create_translation_matrix(
                                [xy[0], xy[1]])*dpg.create_rotation_matrix(course_rad,
                                                                           [0, 0, -1]))
        # Update history
        dpg.set_value(f"tag_hist_{vessel_id}",
                      list(zip(*xy_hist)))

        dpg.configure_item(f"annot_{vessel_id}",
                           default_value=[xy[0],
                                          xy[1]],
                           label=f"{vessel_id}\n"
                           f"speed {speed_kn:.1f}kn \n"
                           f"course {course_deg:.1f}deg")
        # Update waypoints if they've changed
        wps = [list(wp) for wp in list(zip(*waypoints))]
        if dpg.get_value(f"waypoint_plot_{vessel_id}") != wps:
            dpg.set_value(f"waypoint_plot_{vessel_id}",
                          list(zip(*waypoints)))

            wp_x = [x[0] for x in waypoints]
            wp_y = [x[1] for x in waypoints]
            wp_x.insert(0, xy_hist[-1][0])
            wp_y.insert(0, xy_hist[-1][1])
            wp_x_int = list(np.interp(np.linspace(0, len(wp_x), 40),
                                      np.linspace(0, len(wp_x), len(wp_x)),
                                      wp_x))
            wp_y_int = list(np.interp(np.linspace(0, len(wp_y), 40),
                                      np.linspace(0, len(wp_y), len(wp_y)),
                                      wp_y))

            dpg.set_value(f"waypoint_line_{vessel_id}",
                          [wp_x_int,
                           wp_y_int])

    def update_time(self, t):
        dpg.configure_item("time_tag",
                           default_value=f"Time: {np.floor(t/60):.0f}min {t%60:.0f}s")

    def is_plotter_running(self):
        if dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()
        return dpg.is_dearpygui_running()

    def save_init(self):
        print('saving')
        dpg.save_init_file("dpg.ini")

    def set_playspeed(self, sender, app_data):
        self.playspeed = app_data

    def tidy_up(self):
        dpg.destroy_context()
