import dearpygui.dearpygui as dpg
from mass_simulator.agent import Agent, OtherVessel
import os
import numpy as np
from mass_simulator.general import *
from time import time
from logic_bridge import bin_constants, logic_bridge


class Plotter():
    def __init__(self,
                 vessels,
                 xy_lims,
                 control=True):

        self.play = True
        self.playspeed = 10
        self.tic = time()
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
            self._initialise_controls(vessels)

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
            dpg.add_key_press_handler(callback=self._key_press_callback)
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
                         pan_button=dpg.mvMouseButton_Middle,
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

    def _initialise_controls(self, vessels):
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

            with dpg.menu(label="Segmentation layers"):
                for v_key in vessels:
                    dpg.add_checkbox(label=f"{v_key}",
                                     callback=self._add_seg_layers,
                                     user_data=v_key)

    def _add_seg_layers(self, sender, app_data, user_data):
        # If the tag isn't already switched, add it
        dpg.configure_item(item=f"tag_seg_{user_data}",
                           show=app_data)

    def _initialise_segmentation(self,
                                 v_key,
                                 col,
                                 ):

        # Draw the segmentation lines
        with dpg.draw_node(parent="map_plot_tag",
                           tag=f"tag_seg_{v_key}",
                           show=False):

            # Make the lines translucent
            col_tmp = col + [0.3*255]

            # Draw range circles
            for d in bin_constants.RANGE_BINS:
                dpg.draw_circle([0, 0],
                                d[2],
                                color=col_tmp,
                                thickness=0.1,
                                )

            for sect in bin_constants.SECTOR:
                th_1 = logic_bridge.segment_to_bearing(self, segment=sect[1])
                dpg.draw_line([0., 0.],
                              [1000000.*np.cos((th_1[0]*np.pi/180.)),
                               1000000.*np.sin((th_1[0]*np.pi/180.))],
                              color=col_tmp,
                              thickness=0.1,
                              )

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

        # Add an annotation for later use on the mouse cursor
        dpg.add_plot_annotation(label="",
                                default_value=[0, 0],
                                offset=[-5, -5],
                                parent='map_plot_tag',
                                tag=f"tag_annot_seg",
                                show=False)
        # Add a second annotation for later use on the mouse cursor
        dpg.add_plot_annotation(label="",
                                default_value=[0, 0],
                                offset=[5, 5],
                                parent='map_plot_tag',
                                tag=f"tag_annot_cpa",
                                show=False)

        for v_key in vessels:
            v = vessels[v_key]

            # Setup plot colours for the boat
            col = self._setup_plot_themes(n)

            self._initialise_segmentation(v_key,
                                          col)

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
                                    tag=f"v_annot_{v_key}")

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

            # self._set_action_urgency_menu(user_data)
        self._close_wp_menu()

    def _set_action_urgency_menu(self, mouse_pos):
        with dpg.window(label="Timeliness",
                        pos=mouse_pos,
                        no_close=True,
                        no_collapse=True,
                        no_move=True,
                        tag="urgency_menu_tag"):
            dpg.add_button(label='N/A', callback=self._set_action_urgency)
            dpg.add_button(label="Early", callback=self._set_action_urgency)
            dpg.add_button(label="Standard", callback=self._set_action_urgency)
            dpg.add_button(label="Urgent", callback=self._set_action_urgency)

    def _set_action_urgency(self, sender, appdata):
        self.urgency = dpg.get_item_label(sender)
        self._send_waypoints = True
        dpg.delete_item("urgency_menu_tag")

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
        self._waypoints_changed = True

    def _update_waypoint_plot(self):
        # Show waypoints on plot
        for key, value in self._waypoints_temp.items():
            if self._waypoints_temp[key] == []:
                dpg.set_value(f"waypoint_temp_{key}",
                              [[]])
            else:
                dpg.set_value(f"waypoint_temp_{key}",
                              list(zip(*self._waypoints_temp[key])))

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
        return
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
            self._hide_cpa()

        if self._adding_wp != -1:
            self._adding_wp = -1
            self._waypoints_changed = True
            self._hide_cpa()

        if self._select_boat(mouse_pos):
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
                                         -0.1,
                                         max_speed)
            dpg.configure_item("speed_change_tag",
                               label=f"Change speed: \n{self._speed_tmp_kn}kn")

    def _mouse_release_callback(self, sender, app_data):
        self._adjust_speed = False
        if self._speed_tmp_kn > 0:
            self._send_speed_change = True
        dpg.configure_item("speed_change_tag",
                           show=False)

    def _right_click_callback(self, sender, app_data):
        self._close_wp_menu()
        if self._vessel_id_foc not in self._waypoints_temp:
            self._initialise_wp_list(self._vessel_id_foc)

        self._remove_old_wps()

        mouse_pos = dpg.get_plot_mouse_pos()
        mouse_pos2 = dpg.get_mouse_pos()

        with dpg.window(label="Waypoints",
                        pos=mouse_pos2,
                        no_close=True,
                        no_collapse=True,
                        no_move=True,
                        tag="wp_menu_tag"):
            dpg.add_button(label='Send',
                           callback=self._send_wps_cb,
                           user_data=mouse_pos2)

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

    def _key_press_callback(self, sender, app_data):
        if dpg.is_key_pressed(dpg.mvKey_Spacebar):
            self.set_play(not self.play)

    def _initialise_wp_list(self, vessel_id):
        wp = dpg.get_value(f"waypoint_plot_{vessel_id}")
        wp_n = dpg.get_item_user_data(f"waypoint_plot_{vessel_id}")

        v_xy = [dpg.get_value(f"tag_hist_{vessel_id}")[0][-1],
                dpg.get_value(f"tag_hist_{vessel_id}")[1][-1]]
        self._waypoints_temp[vessel_id] = list(zip(*wp))[wp_n:]
        self._waypoints_temp[vessel_id].insert(0, v_xy)

    def _remove_old_wps(self):
        for v_id in self._waypoints_temp.keys():
            waypoints = dpg.get_value(f"waypoint_plot_{v_id}")
            wp_n = dpg.get_item_user_data(f"waypoint_plot_{v_id}")
            for waypoint in list(zip(*waypoints))[0:wp_n]:
                wp_tmp = [w for w in self._waypoints_temp[v_id]
                          if w != waypoint]
                self._waypoints_temp[v_id] = wp_tmp

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
            self._update_vessel_plot(vessel=v)
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

            t_s = abs(np.sign(v.tcpa_s)*v.tcpa_s % 60)
            dpg.set_value(f"tag_tcpa_v{n}",
                          f"{np.floor(v.tcpa_s/60):.0f}min {t_s:.0f}s")
            dpg.set_value(f"tag_range_v{n}", f"{v.range_yds:.0f}")
            dpg.set_value(f"tag_bearing_v{n}", f"{v.bearing_deg:.1f}")
            n += 1

    def _update_vessel_plot(self,
                            vessel: Agent):

        vessel_id = vessel.vessel_id
        xy = vessel.xy
        xy_hist = vessel.xy_hist
        course_deg = vessel.course_deg
        speed_kn = vessel.speed_kn
        speed_mps = vessel.speed_mps
        waypoints = vessel.waypoints
        activity = vessel.activity
        wp_n = vessel.waypoint_n

        course_rad = (course_deg*np.pi/180.)
        dpg.apply_transform(f"tag_seg_{vessel_id}",
                            transform=dpg.create_translation_matrix(
                                [xy[0], xy[1]])*dpg.create_rotation_matrix(course_rad,
                                                                           [0, 0, -1]))
        dpg.apply_transform(f"tag_draw_{vessel_id}",
                            transform=dpg.create_translation_matrix(
                                [xy[0], xy[1]])*dpg.create_rotation_matrix(course_rad,
                                                                           [0, 0, -1]))
        # Update history
        dpg.set_value(f"tag_hist_{vessel_id}",
                      list(zip(*xy_hist)))

        dpg.configure_item(f"v_annot_{vessel_id}",
                           default_value=[xy[0],
                                          xy[1]],
                           label=f"{vessel_id}\n"
                           f"speed {speed_kn:.1f}kn \n"
                           f"course {course_deg:.1f}deg \n"
                           f"{activity}",
                           user_data=xy+[course_deg]+[speed_mps])
        # Update waypoints if they've changed
        wps = [list(wp) for wp in list(zip(*waypoints))]
        dpg.set_item_user_data(f"waypoint_plot_{vessel_id}",
                               wp_n)
        if dpg.get_value(f"waypoint_plot_{vessel_id}") != wps:
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

    def _update_seg_annot(self, mouse_pos):

        tag_segs = [x for x in dpg.get_aliases() if "tag_seg_" in x]

        label_tmp = ""
        for t in tag_segs:
            if dpg.get_item_configuration(t)["show"]:
                v_id = t[8:]
                v_xy_course = dpg.get_item_user_data(f"v_annot_{v_id}")
                # compute range segment
                d = np.sqrt(np.square(mouse_pos[0]-v_xy_course[0])
                            + np.square(mouse_pos[1]-v_xy_course[1]))
                d_seg = [b[0] for b in bin_constants.RANGE_BINS if d >
                         b[1] and d < b[2]][0]
                # compute bearing segment
                brg_deg = (np.atan2(mouse_pos[0]-v_xy_course[0],
                                    mouse_pos[1]-v_xy_course[1])*180/np.pi) % 360
                brg_deg_adj = brg_deg - v_xy_course[2]
                lb = logic_bridge()
                brg_sect = lb.bearing_to_sector(brg_deg_adj)
                label_tmp += f"{v_id}: {d_seg} on {brg_sect}\n"

        if label_tmp != "":
            dpg.configure_item(f"tag_annot_seg",
                               default_value=[mouse_pos[0],
                                              mouse_pos[1]],
                               label=label_tmp,
                               show=True)
        else:
            dpg.configure_item(f"tag_annot_seg",
                               show=False)

    def _compute_new_cpa(self, mouse_pos):
        # Get focussed vessel state
        xy_course_speed = dpg.get_item_user_data(
            f"v_annot_{self._vessel_id_foc}")
        xy1 = xy_course_speed[0:2]
        speed_mps1 = xy_course_speed[3]
        course_deg_new = 90 - np.atan2(mouse_pos[1]-xy1[1],
                                       mouse_pos[0]-xy1[0])*180/np.pi

        # Get forecast CPA to every other vessel
        v_other = [v for v in dpg.get_aliases()
                   if "v_annot_" in v]
        label_tmp = "New CPA: \n"
        for v in v_other:
            if v != f"v_annot_{self._vessel_id_foc}":
                xy_course_speed2 = dpg.get_item_user_data(v)
                _, cpa_yds, _ = compute_cpa(xy1=xy1,
                                            course1=course_deg_new,
                                            speed_mps1=speed_mps1,
                                            xy2=xy_course_speed2[0:2],
                                            course2=xy_course_speed2[2],
                                            speed_mps2=xy_course_speed2[3])
                label_tmp += f"{v[8:]}: {cpa_yds:.0f}yds \n"

        dpg.configure_item(f"tag_annot_cpa",
                           default_value=[mouse_pos[0],
                                          mouse_pos[1]],
                           label=label_tmp,
                           show=True)

    def _hide_cpa(self):
        dpg.configure_item(f"tag_annot_cpa",
                           show=False)

    def _dynamic_waypoint_move(self):

        mouse_pos = dpg.get_plot_mouse_pos()

        # If the segmentation layer is on, show the segment next to the cursor
        self._update_seg_annot(mouse_pos=mouse_pos)
        if self._waypoints_temp != {}:
            for key, value in self._waypoints_temp.items():
                # Set the first value to the current position so it tracks the boat
                xy_hist = dpg.get_value(f"tag_hist_{key}")
                xy_curr = [xy_hist[0][-1],
                           xy_hist[1][-1]]
                value[0] = xy_curr
                self._update_waypoint_plot()

        if self._changing_wp != -1:
            wp_n = self._changing_wp
            self._add_or_change_wps(wp_n, mouse_pos)
        elif self._adding_wp != -1:
            wp_n = self._adding_wp+1
            self._add_or_change_wps(wp_n, mouse_pos)

    def _add_or_change_wps(self, wp_n, mouse_pos):
        self._waypoints_temp[self._vessel_id_foc][wp_n] = mouse_pos
        self._update_waypoint_plot()
        wp_current = dpg.get_item_user_data(
            f"waypoint_plot_{self._vessel_id_foc}")
        if wp_n == wp_current or wp_n == 1:
            self._compute_new_cpa(mouse_pos)

    def save_init(self):
        print('saving')
        dpg.save_init_file("dpg.ini")

    def set_playspeed(self, sender, app_data):
        self.playspeed = app_data

    def tidy_up(self):
        dpg.destroy_context()

    def advance_one_frame_check(self, t_step):
        if time() - self.tic >= t_step/(self.playspeed):
            self.tic = time()
            return True
        else:
            return False
