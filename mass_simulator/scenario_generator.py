import numpy as np
import dearpygui.dearpygui as dpg
import json
import utm
from general import *
import matplotlib.cm as cm


class ScenarioGenerator():
    def __init__(self):

        self._scen_dict = {"params": {"t_step": 1},
                           "vessel_details": []}
        self._vessel_n = 0
        self._adding_wps = False
        self._setting_depth = False

        dpg.create_context()

        # Put stuff in here

        self._initialise_map()
        self._add_depth_map(50000, 30000)
        self._setup_click_handlers()

        dpg.set_primary_window("world_window", True)

        dpg.create_viewport(title="MASS Simulator",
                            width=1000,
                            height=1000)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()

    def _setup_click_handlers(self):
        with dpg.item_handler_registry(tag="click_handler"):
            dpg.add_item_clicked_handler(callback=self._left_click_callback,
                                         button=dpg.mvMouseButton_Left)
            dpg.add_item_clicked_handler(callback=self._middle_click_callback,
                                         button=dpg.mvMouseButton_Middle)
            dpg.add_item_clicked_handler(callback=self._right_click_callback,
                                         button=dpg.mvMouseButton_Right)
        dpg.bind_item_handler_registry(item="map_plot_tag",
                                       handler_registry="click_handler")

    def _left_click_callback(self):
        self._close_rightclick_menu()

        if self._adding_wps:
            mouse_pos = dpg.get_plot_mouse_pos()
            self._add_wp(mouse_pos)

    def _middle_click_callback(self):
        self._close_rightclick_menu()

    def _right_click_callback(self):
        self._close_rightclick_menu()

        mouse_pos = dpg.get_mouse_pos()
        mouse_pos_plot = dpg.get_plot_mouse_pos()
        with dpg.window(label="Add",
                        no_title_bar=True,
                        pos=mouse_pos,
                        tag="rightclick_menu_tag",
                        no_close=True,
                        no_move=True,
                        no_collapse=True):
            dpg.add_button(label="Add vessel here",
                           callback=self._add_new_boat,
                           user_data=[mouse_pos,
                                      mouse_pos_plot])
            with dpg.group(horizontal=True):
                dpg.add_button(label="Adjust topography",
                               callback=self._set_depth,
                               user_data=[mouse_pos,
                                          mouse_pos_plot])
                depth_show = dpg.get_item_configuration("depth_img_tag")[
                    "show"]
                dpg.add_checkbox(label="Show depth",
                                 callback=self._show_depth_map,
                                 default_value=depth_show)
            dpg.add_button(label="Save scenario",
                           callback=self._save_callback)

    def _show_depth_map(self, sender, app_data):
        dpg.configure_item("depth_img_tag",
                           show=app_data)

    def _initialise_map(self):
        with dpg.window(label="World",
                        tag="world_window",
                        no_close=True,
                        autosize=True):
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

    def _add_vessel_plot(self, vessel_pos):
        # Get colour
        col = self._setup_plot_themes(self._vessel_n)
        with dpg.draw_node(parent="map_plot_tag",
                           tag=f'tag_draw_{self._vessel_n}'):
            dpg.draw_triangle(p1=[-200, -200],
                              p2=[0, 400],
                              p3=[200, -200],
                              color=col,
                              fill=col,
                              thickness=0.1)
        dpg.apply_transform(f'tag_draw_{self._vessel_n}',
                            transform=dpg.create_translation_matrix(
                                [vessel_pos[0], vessel_pos[1]])*dpg.create_rotation_matrix(0,
                                                                                           [0, 0, -1]))
        dpg.add_line_series(x=[vessel_pos[0]],
                            y=[vessel_pos[1]],
                            parent="map_y_axis",
                            tag=f"waypoint_plot_{self._vessel_n}")

        dpg.add_plot_annotation(label=f"vessel{self._vessel_n}",
                                parent="map_plot_tag",
                                default_value=vessel_pos,
                                offset=[10, 0],
                                tag=f"tag_label_{self._vessel_n}")

        # Change the colours of the plots
        dpg.bind_item_theme(f"waypoint_plot_{self._vessel_n}",
                            f'waypoint_theme_{self._vessel_n}')

    def _add_depth_map(self, width, height):
        self._depth_points = [[[0, 0], 1],
                              [[width, 0], 1],
                              [[width, height], 1],
                              [[0, height], 1]]

        self._update_depth_map()
        for n, d in enumerate(self._depth_points):
            dpg.add_plot_annotation(label=d[1],
                                    default_value=d[0],
                                    parent="map_plot_tag",
                                    tag=f"annot_{n}")

    def _update_depth_map(self):
        depth_map_norm_fl, p_min, p_max = compute_interp_depth_map(
            self._depth_points)

        width = p_max[1]-p_min[1]
        height = p_max[0]-p_min[0]

        if "depth_img_tag" in dpg.get_aliases():
            dpg.delete_item("depth_img_tag")
            dpg.delete_item("depth_map_texture")

        with dpg.texture_registry(show=False):
            dpg.add_static_texture(width=round(width/100),
                                   height=round(height/100),
                                   default_value=cm.RdYlGn(depth_map_norm_fl),
                                   tag="depth_map_texture")

        dpg.add_image_series(texture_tag="depth_map_texture",
                             bounds_min=[p_min[1], p_min[0]],
                             bounds_max=[p_max[1], p_max[0]],
                             parent="map_y_axis",
                             tag="depth_img_tag")

    def _add_new_boat(self, sender, appdata, user_data):
        self._close_rightclick_menu()

        default_vals = {"label": f"vessel{self._vessel_n}",
                        "speed_kn": 10,
                        "draught_m": 12,
                        "turning_radius_m": 300,
                        "speedchange_rate": 0.015,
                        "max_speed": 25}

        self._add_vessel_plot(user_data[1])
        with dpg.window(label="Vessel Parameters",
                        tag=f"vessel_params_{self._vessel_n}"):
            dpg.add_input_text(label="Vessel label",
                               callback=self._add_boat_details,
                               user_data='vessel')
            dpg.add_input_double(label="Speed, knots",
                                 default_value=default_vals["speed_kn"],
                                 width=100,
                                 callback=self._add_boat_details,
                                 user_data="speed_kn")
            dpg.add_input_double(label="Draught, m",
                                 default_value=default_vals["draught_m"],
                                 width=100,
                                 callback=self._add_boat_details,
                                 user_data="draught_m")
            dpg.add_input_double(label="Turning Radius, m",
                                 default_value=default_vals["turning_radius_m"],
                                 width=100,
                                 callback=self._add_boat_details,
                                 user_data="turning_radius")
            dpg.add_input_double(label="Rate of change of speed, kn/s",
                                 default_value=default_vals["speedchange_rate"],
                                 width=100,
                                 callback=self._add_boat_details,
                                 user_data="speed_change_knps")
            dpg.add_input_double(label="Max speed, kn",
                                 default_value=default_vals["max_speed"],
                                 width=100,
                                 callback=self._add_boat_details,
                                 user_data="speed_max_kn")
            dpg.add_separator()
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="Add waypoints",
                                 callback=self._add_wps_check)
                dpg.add_button(label="Done",
                               callback=self._done_adding_boat)

        self._waypoints_temp = [user_data[1]]
        self._adding_wps = False

        self._scen_dict['vessel_details'].append(
            {"vessel": default_vals["label"],
             "speed_kn": default_vals["speed_kn"],
             "draught_m": default_vals["draught_m"],
             "turning_radius": default_vals["turning_radius_m"],
             "speed_change_knps": default_vals["speedchange_rate"],
             "speed_max_kn": default_vals["max_speed"]})

    def _add_wps_check(self, sender, app_data):
        self._adding_wps = app_data

    def _add_wp(self, xy):
        self._waypoints_temp.append(xy)
        if len(self._waypoints_temp) == 2:
            xy0 = self._waypoints_temp[0]
            xy1 = self._waypoints_temp[1]
            brg = np.deg2rad(compute_bearing(xy0, xy1))
            dpg.apply_transform(f'tag_draw_{self._vessel_n}',
                                transform=dpg.create_translation_matrix(
                                    [xy0[0], xy0[1]])*dpg.create_rotation_matrix(brg,
                                                                                 [0, 0, -1]))

        dpg.set_value(f"waypoint_plot_{self._vessel_n}",
                      list(zip(*self._waypoints_temp)))

    def _add_boat_details(self, sender, app_data, user_data):
        self._scen_dict['vessel_details'][self._vessel_n][user_data] = app_data

        if user_data == "vessel":
            dpg.configure_item(f"tag_label_{self._vessel_n}",
                               label=app_data)

    def _done_adding_boat(self, sender, app_data, user_data):
        self._scen_dict['vessel_details'][self._vessel_n]["waypoints"] = self._waypoints_temp
        if f"vessel_params_{self._vessel_n}" in dpg.get_aliases():
            dpg.delete_item(f"vessel_params_{self._vessel_n}")
        print(self._scen_dict)
        self._vessel_n += 1
        self._adding_wps = False

    def _define_new_area(self):
        pass

    def _set_depth(self, sender, app_data, user_data):
        self._close_rightclick_menu()

        xy = user_data[1]

        with dpg.window(modal=True,
                        popup=True,
                        no_close=True,
                        no_move=True,
                        no_collapse=True,
                        no_title_bar=True,
                        autosize=True,
                        tag="depth_input_window"):
            dpg.add_drag_float(label="height",
                               tag="depth_input_tag",
                               min_value=-2000,
                               max_value=1)
            dpg.add_button(label="Set",
                           callback=self._add_new_depth,
                           user_data=xy)

    def _add_new_depth(self, sender, app_data, userdata):
        depth = dpg.get_value("depth_input_tag")

        new_d = True
        for n, depth_p in enumerate(self._depth_points):
            d = compute_distance(depth_p[0], userdata)
            if d < 700:
                self._depth_points[n][1] = depth
                dpg.configure_item(f"annot_{n}",
                                   label=depth)
                new_d = False
                break
        if new_d == True:
            self._depth_points.append([userdata, depth])
            n_len = len(self._depth_points)-1
            dpg.add_plot_annotation(label=depth,
                                    default_value=userdata,
                                    parent="map_plot_tag",
                                    tag=f"annot_{n_len}")
        self._update_depth_map()
        dpg.delete_item("depth_input_window")

    def _save_callback(self):
        self._close_rightclick_menu()

        # Open directory selector
        dpg.add_file_dialog(callback=self._save_file)

    def _save_file(self, sender, appdata):
        if ".json" not in appdata['file_path_name']:
            file_pth = appdata["file_path_name"] + ".json"
        else:
            file_pth = appdata["file_path_name"]

        # Add the depth map
        self._scen_dict['depth_map'] = self._depth_points

        with open(file_pth, 'w') as f:
            # write the dict to a json file
            json.dump(self._scen_dict,
                      f,
                      indent=4)

    def _close_rightclick_menu(self):
        if "rightclick_menu_tag" in dpg.get_aliases():
            dpg.delete_item("rightclick_menu_tag")

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

        with dpg.theme(tag=f'waypoint_theme_{n}'):
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line,
                                    col,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
                                    dpg.mvPlotMarker_Cross,
                                    category=dpg.mvThemeCat_Plots)
        # with dpg.theme(tag="areas_theme"):
        #     with dpg.theme_component(dpg.mvScatterSeries):
        #         dpg.add_theme_color(dpg.mvPlotCol_Line,
        #                             col,
        #                             category=dpg.mvThemeCat_Plots)
        #         dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
        #                             dpg.mvPlotMarker_Cross,
        #                             category=dpg.mvThemeCat_Plots)
        #         dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize,
        #                             6,
        #                             category=dpg.mvThemeCat_Plots)

        return col

    def _is_existing_wp(self, xy):
        d = [compute_distance(xy, xy2) for xy2
             in self._waypoints_temp]

        n_close = np.where(np.array(d) < 500)[0].tolist()
        if n_close == []:
            close_wp = False
        else:
            close_wp = True
        return close_wp, n_close


def main():
    scen_gen = ScenarioGenerator()


if __name__ == '__main__':
    main()
