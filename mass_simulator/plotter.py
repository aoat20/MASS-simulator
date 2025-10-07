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
        self._adjust_speed = False
        self._speed_tmp_kn = 0
        self._send_speed_change = False
        self._waypoints_temp = {}
        self._send_waypoints = False
        self._changing_wp = -1
        self._adding_wp = -1
        self._waypoints_changed = False
        self._vessel_id_foc = next(iter(vessels))
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
        self._set_event_handlers()

        self._add_vessels(vessels=vessels)
        self._initialise_variable_viewer(vessel_N)
        self._change_vessel_plot()

        if control:
            self._initialise_controls()

        dpg.create_viewport(title="MASS Simulator",
                            width=600,
                            height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def _set_event_handlers(self):
        with dpg.item_handler_registry(tag="click_handler"):
            dpg.add_item_clicked_handler(callback=self._left_click_callback,
                                         button=dpg.mvMouseButton_Left)
            dpg.add_item_clicked_handler(callback=self._middle_click_callback,
                                         button=dpg.mvMouseButton_Middle)
            dpg.add_item_clicked_handler(callback=self._right_click_callback,
                                         button=dpg.mvMouseButton_Right)

        with dpg.handler_registry():
            dpg.add_mouse_drag_handler(callback=self._mouse_drag_callback,
                                       button=dpg.mvMouseButton_Left)
            dpg.add_mouse_release_handler(callback=self._mouse_release_callback,
                                          button=dpg.mvMouseButton_Left)
        dpg.bind_item_handler_registry(item="map_plot_tag",
                                       handler_registry="click_handler")

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

        with dpg.theme(tag=f'hist_theme_{n}'):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight,
                                    4,
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

        with dpg.theme(tag=f"current_wp_theme_{n}"):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
                                    dpg.mvPlotMarker_Diamond,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize,
                                    6,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight,
                                    2,
                                    category=dpg.mvThemeCat_Plots)

        with dpg.theme(tag=f'new_wp_theme_{n}'):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
                                    dpg.mvPlotMarker_Diamond,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize,
                                    5,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_FillAlpha,
                                    0,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight,
                                    1,
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
                         no_menus=True,
                         no_box_select=True,
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
                        tag="tag_control",
                        min_size=[250, 140],
                        max_size=[250, 140]):
            dpg.add_checkbox(label="Play",
                             callback=self._set_play,
                             default_value=True,
                             tag='tag_play')
            dpg.add_drag_float(label="Playspeed",
                               min_value=0.1,
                               max_value=100,
                               default_value=self.playspeed,
                               callback=self.set_playspeed)

    def add_time_scrubber(self, t_max):
        with dpg.window(label='Time',
                        min_size=[1100, 70],
                        max_size=[1100, 70]):
            dpg.add_slider_float(label="Time, s",
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
        self.update_time(t)

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

    def get_waypoint_updates(self):
        if self._send_waypoints \
                and self._waypoints_temp != {}:
            wp_ret = {}
            for key in self._waypoints_temp:
                wp_ret[key] = self._waypoints_temp[key][1:]
            self._waypoints_temp = {}
            self._clear_wps([], [])
            self._send_waypoints = False
            self._waypoints_changed = False
            return wp_ret
        else:
            return []

    def get_speed_change(self):
        speed_mps = self._speed_tmp_kn*0.514444
        return self._send_speed_change, self._vessel_id_foc, speed_mps

    def _initialise_variable_viewer(self,
                                    vessels_N):
        with dpg.window(label="Vessel States",
                        no_close=True,
                        min_size=[110+vessels_N*110, 168],
                        max_size=[110+vessels_N*110, 168]):
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
                    dpg.add_text('Range, yds')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_range_v{n}")
                with dpg.table_row():
                    dpg.add_text('Bearing, deg')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_bearing_v{n}")
                with dpg.table_row():
                    dpg.add_text('CPA, yds')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_cpa_v{n}")
                with dpg.table_row():
                    dpg.add_text('TCPA')
                    for n in range(vessels_N-1):
                        dpg.add_text(tag=f"tag_tcpa_v{n}")

    def _add_vessels(self, vessels):
        v: Agent
        n = 0

        # Add hidden annotation for speed change
        dpg.add_plot_annotation(label="",
                                default_value=[0, 0],
                                offset=[-5, -5],
                                parent='map_plot_tag',
                                tag=f"speed_change_tag",
                                show=False)

        for v_key in vessels:
            v = vessels[v_key]

            # Setup plot colours for the boat
            col = self._setup_plot_themes(n)

            # Set up line for history
            dpg.add_line_series(label=v_key,
                                x=[v.xy[0]],
                                y=[v.xy[1]],
                                parent='map_y_axis',
                                tag=f"tag_hist_{v_key}")

            # Set up triangle for vessel locations
            with dpg.draw_node(parent="map_plot_tag",
                               tag=f"tag_draw_{v_key}"):
                dpg.draw_triangle(p1=[-200, -200],
                                  p2=[0, 400],
                                  p3=[200, -200],
                                  color=col,
                                  fill=col,
                                  thickness=0.1,
                                  tag=f"tag_triangle_filled_{v_key}",
                                  user_data=v.max_speed_kn,
                                  show=False)
                dpg.draw_triangle(p1=[-200, -200],
                                  p2=[0, 400],
                                  p3=[200, -200],
                                  color=col,
                                  thickness=1,
                                  tag=f"tag_triangle_{v_key}",
                                  user_data=v.max_speed_kn)

            # Add annotation for vessel information
            dpg.add_plot_annotation(label=f"{v_key}\n"
                                    "speed -kn \n"
                                    "course -deg",
                                    default_value=[0, 0],
                                    offset=[5, 5],
                                    parent='map_plot_tag',
                                    tag=f"annot_{v_key}")

            # Add crosses for waypoints
            dpg.add_line_series(x=[],
                                y=[],
                                parent="map_y_axis",
                                tag=f"waypoint_plot_{v_key}")
            dpg.add_line_series(x=[],
                                y=[],
                                parent="map_y_axis",
                                tag=f"waypoint_temp_{v_key}")

            # Add original waypoints
            dpg.add_scatter_series(label=f"{v_key} original waypoints",
                                   x=[xy[0] for xy in v.waypoints],
                                   y=[xy[1] for xy in v.waypoints],
                                   tag=f"waypoint_orig_{v_key}",
                                   parent="map_y_axis")

            # Change the colours of the plots
            dpg.bind_item_theme(f"tag_hist_{v_key}",
                                f'hist_theme_{n}')
            dpg.bind_item_theme(f"waypoint_orig_{v_key}",
                                f'hist_theme_{n}')
            dpg.bind_item_theme(f"waypoint_plot_{v_key}",
                                f'current_wp_theme_{n}')
            dpg.bind_item_theme(f"waypoint_temp_{v_key}",
                                f'new_wp_theme_{n}')

            n += 1

    def _change_vessel_plot(self):
        for v_id in dpg.get_aliases():
            if "tag_triangle_filled_" in v_id:
                dpg.configure_item(v_id,
                                   show=False)
            dpg.configure_item(f"tag_triangle_filled_{self._vessel_id_foc}",
                               show=True)

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
                    self._change_vessel_plot()
                    return True

    def _add_temporary_waypoints(self, xy):
        # update the waypoints of the current focussed vessel
        if self._vessel_id_foc not in self._waypoints_temp:
            self._waypoints_temp[self._vessel_id_foc] = []

        close_bool, n_close = self._is_existing_wp(xy)
        if close_bool:
            for n in n_close[::-1]:
                self._waypoints_temp[self._vessel_id_foc].pop(n)
        else:
            self._waypoints_temp[self._vessel_id_foc].append(xy)

        self._waypoints_changed = True
        self._update_waypoint_plot()

    def _send_wps_cb(self, sender, app_data, user_data):
        if self._waypoints_changed:
            self._send_waypoints = True
        self._close_wp_menu()

    def _add_waypoint(self, sender, appdata, user_data):
        self._adding_wp = user_data
        self._close_wp_menu()
        mouse_pos = dpg.get_plot_mouse_pos()

        self._waypoints_temp[self._vessel_id_foc].insert(
            self._adding_wp+1,
            mouse_pos)

    def _change_waypoint(self, sender, appdata, user_data):
        self._changing_wp = user_data[0]
        self._close_wp_menu()

    def _remove_waypoint(self, sender, app_data, user_data):
        n = user_data[0]
        self._waypoints_temp[self._vessel_id_foc].pop(n)
        self._update_waypoint_plot()
        self._close_wp_menu()

    def _update_waypoint_plot(self):
        # Show waypoints on plot
        if self._waypoints_temp[self._vessel_id_foc] == []:
            dpg.set_value(f"waypoint_temp_{self._vessel_id_foc}",
                          [[]])
        else:
            dpg.set_value(f"waypoint_temp_{self._vessel_id_foc}",
                          list(zip(*self._waypoints_temp[self._vessel_id_foc])))

    def _is_existing_wp(self, xy):
        d = [compute_distance(xy, xy2) for xy2
             in self._waypoints_temp[self._vessel_id_foc]]

        n_close = np.where(np.array(d) < 500)[0].tolist()
        if n_close == []:
            close_wp = False
        else:
            close_wp = True
        return close_wp, n_close

    def _is_on_current_wps(self, xy):
        for wp_n in range(len(self._waypoints_temp[self._vessel_id_foc])-1):
            wp1 = self._waypoints_temp[self._vessel_id_foc][wp_n]
            wp2 = self._waypoints_temp[self._vessel_id_foc][wp_n+1]
            d = compute_perp_distance(wp1,
                                      wp2,
                                      xy)
            if d < 500:
                return True, wp_n

        return False, -1

    def _middle_click_callback(self, sender, app_data):
        mouse_pos = dpg.get_plot_mouse_pos()
        if not self._select_boat(mouse_pos):
            self._send_waypoints = False
            self._add_temporary_waypoints(mouse_pos)
        else:
            self._send_waypoints = True

    def _left_click_callback(self, sender, app_data):
        self._close_wp_menu()
        mouse_pos = dpg.get_plot_mouse_pos()

        if self._changing_wp != -1:
            self._changing_wp = -1
            self._waypoints_changed = True

        if self._adding_wp != -1:
            self._adding_wp = -1
            self._waypoints_changed = True

        if self._select_boat(mouse_pos):
            dpg.configure_item('map_plot_tag',
                               pan_button=dpg.mvMouseButton_Right)
            self._adjust_speed = True
            dpg.configure_item("speed_change_tag",
                               default_value=mouse_pos,
                               show=True)
            return

    def _mouse_drag_callback(self, sender, app_data):
        if self._adjust_speed:
            # get max speed
            max_speed = dpg.get_item_user_data(
                f"tag_triangle_{self._vessel_id_foc}")

            self._speed_tmp_kn = np.clip(-app_data[2]/10,
                                         0,
                                         max_speed)
            dpg.configure_item("speed_change_tag",
                               label=f"Change speed: \n{self._speed_tmp_kn}kn")

    def _mouse_release_callback(self, sender, app_data):
        dpg.configure_item('map_plot_tag',
                           pan_button=dpg.mvMouseButton_Left)
        self._adjust_speed = False
        if self._speed_tmp_kn > 1:
            self._send_speed_change = True
        dpg.configure_item("speed_change_tag",
                           show=False)

    def _right_click_callback(self, sender, app_data):
        self._close_wp_menu()
        if self._vessel_id_foc not in self._waypoints_temp:
            wp = dpg.get_value(f"waypoint_plot_{self._vessel_id_foc}")
            wp_n = dpg.get_item_user_data(
                f"waypoint_plot_{self._vessel_id_foc}")

            v_xy = [dpg.get_value(f"tag_hist_{self._vessel_id_foc}")[0][-1],
                    dpg.get_value(f"tag_hist_{self._vessel_id_foc}")[1][-1]]
            self._waypoints_temp[self._vessel_id_foc] = list(zip(*wp))[wp_n:]
            self._waypoints_temp[self._vessel_id_foc].insert(0, v_xy)

        mouse_pos = dpg.get_plot_mouse_pos()
        mouse_pos2 = dpg.get_mouse_pos()

        with dpg.window(label="Waypoints",
                        pos=mouse_pos2,
                        no_close=True,
                        no_collapse=True,
                        no_move=True,
                        tag="wp_menu_tag"):
            dpg.add_button(label='Send',
                           callback=self._send_wps_cb)

            wp_bool, wp_close = self._is_existing_wp(mouse_pos)
            wp_line_bool, wp_n = self._is_on_current_wps(mouse_pos)
            if wp_bool:
                dpg.add_button(label='Change',
                               callback=self._change_waypoint,
                               user_data=wp_close)
                dpg.add_button(label='Remove',
                               callback=self._remove_waypoint,
                               user_data=wp_close)
            elif wp_line_bool:
                dpg.add_button(label='Add',
                               callback=self._add_waypoint,
                               user_data=wp_n)
            else:
                n_wp = len(self._waypoints_temp[self._vessel_id_foc])-1
                dpg.add_button(label='Add',
                               callback=self._add_waypoint,
                               user_data=n_wp)

    def _close_wp_menu(self):
        if "wp_menu_tag" in dpg.get_aliases():
            dpg.delete_item("wp_menu_tag")

    def reset_speed_change(self):
        self._speed_tmp_kn = 0
        dpg.configure_item("speed_change_tag",
                           label=f"Change speed: \n{self._speed_tmp_kn}kn")
        self._send_speed_change = False

    def update_vessels(self,
                       vessels: dict):
        v: Agent
        for v in vessels.values():
            self._update_vessel_plot(v.vessel_id,
                                     v.xy,
                                     v.xy_hist,
                                     v.course_deg,
                                     v.speed_kn,
                                     v.waypoints,
                                     v.activity,
                                     v.waypoint_n)
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
                            waypoints,
                            activity,
                            wp_n):
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
                           f"course {course_deg:.1f}deg \n"
                           f"{activity}")
        # Update waypoints if they've changed
        wps = [list(wp) for wp in list(zip(*waypoints))]
        if dpg.get_value(f"waypoint_plot_{vessel_id}") != wps:
            dpg.set_item_user_data(f"waypoint_plot_{vessel_id}",
                                   wp_n)
            dpg.set_value(f"waypoint_plot_{vessel_id}",
                          list(zip(*waypoints)))

    def update_time(self, t):
        dpg.configure_item("time_tag",
                           default_value=f"Time: {np.floor(t/60):.0f}min {t%60:.0f}s")

    def is_plotter_running(self):
        if dpg.is_dearpygui_running():
            self._dynamic_waypoint_move()
            dpg.render_dearpygui_frame()
        return dpg.is_dearpygui_running()

    def _dynamic_waypoint_move(self):
        mouse_pos = dpg.get_plot_mouse_pos()

        if self._changing_wp != -1:
            self._waypoints_temp[self._vessel_id_foc][self._changing_wp] = mouse_pos
            self._update_waypoint_plot()
        elif self._adding_wp != -1:
            self._waypoints_temp[self._vessel_id_foc][self._adding_wp+1] = mouse_pos
            self._update_waypoint_plot()

    def save_init(self):
        print('saving')
        dpg.save_init_file("dpg.ini")

    def set_playspeed(self, sender, app_data):
        self.playspeed = app_data

    def tidy_up(self):
        dpg.destroy_context()
