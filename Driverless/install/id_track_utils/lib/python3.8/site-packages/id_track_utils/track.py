from __future__ import annotations

import csv
import dataclasses
import io
import itertools
import math
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union
import xml.etree.ElementTree
from xml.etree.ElementTree import Element, SubElement

# TODO import imperial_driverless_interfaces.msg
# TODO import geometry_msgs.msg

import PIL.Image
import PIL.ImageDraw
import PIL.ImageOps

from pathlib import Path
import yaml

from .track_generator import (  # noqa: E402
    cone_start,
    get_cone_function,
    CONE_INNER, CONE_OUTER,
    CONE_ORANGE, CONE_BIG_ORANGE,
    CONE_START,
    GeneratorContext, TrackGenerator,
    random
)

Cone = Tuple[float, float]

@dataclasses.dataclass
class Track:
    """
    A universal intermediate track representation. Can be used to 
    convert between many different track representations.

    Standard usage: Create a Track object using one of the static methods (e.g. Track.from_csv,
    Track.from_gazebo_sdf, Track.from_conemap_message), and then convert it to a different
    format using the as_* methods.

    Example usage to convert from gazebo sdf to csv: 
    `Track.from_gazebo_sdf(sdf_file_contents).as_csv()`

    Attributes
    ----------
    start_x : float
        x location of car in map.
    start_y : float
        y location of car in map.
    start_yaw : float
        Yaw of car.

    left_cones : List[Cone]
        List of blue cones.
    right_cones : List[Cone]
        List of yellow cones.
    orange_cones : List[Cone]
        List of small orange cones.
    big_orange_cones : List[Cone]
        List of large orange cones.

    frame_id : str
        Type of track
    """
    start_x: float
    start_y: float
    start_yaw: float
    
    left_cones: List[Cone]
    right_cones: List[Cone]
    orange_cones: List[Cone]
    big_orange_cones: List[Cone]

    frame_id: str = 'map'

    @staticmethod
    def generate_random(seed: Optional[int]=None) -> Track:
        prev_random_state = random.getstate()
        if seed is not None:
            random.seed(seed)

        generator_values = TrackGenerator.get_preset('Small Straights') # type: ignore

        def failure_function():
            raise RuntimeError("Random Track generation failed")

        with GeneratorContext(generator_values, failure_function): # type: ignore
            components, track_width, track_height = TrackGenerator.generate()# type: ignore
        
            cone_locs: List[Tuple[float, float, Any]] = []

            cone_locs.extend(cone_start(components[0][1]))

            for name, points in components[1:]:# type: ignore
                cone_locs.extend(get_cone_function(name)(points, prev_points=cone_locs))# type: ignore
            
        # In order for groupby to work, all items that are to be grouped together
        # must be in one contiguous part of the list. Normally we would just sort the
        # list to achieve this, but we're groupping by 'Type' objects, which don't 
        # have a comparison operator. As a workaround, we sort by the type of the item,
        # which ensures that all objects of the same type end up in one contiguous part of the list.
        def sort_key(item: Tuple[float, float, Any]) -> str: return str(item[2])
        cone_locs.sort(key=sort_key)
        g = itertools.groupby(cone_locs, lambda x: x[2])

        cone_lists = {cone_type: [(x, y) for x, y, _ in group] for cone_type, group in g}

        xys = cone_lists[CONE_START]
        sx, sy = xys[0]
        ex, ey = xys[1]

        # So we calculate the angle that the car is facing (the yaw)
        angle = math.atan2(ey - sy, ex - sx)
        if angle < 0:
            # Angle is on range [-pi,pi] but we want [0,2pi]
            # So if it is less than 0, pop it onto the correct range.
            angle += 2 * math.pi

        # Now we get the car position
        car_x, car_y = (sx, sy)

        random.setstate(prev_random_state)

        return Track(car_x, car_y, angle,
            left_cones=cone_lists.get(CONE_INNER, []),
            right_cones=cone_lists.get(CONE_OUTER, []),
            orange_cones=cone_lists.get(CONE_ORANGE, []),
            big_orange_cones=cone_lists.get(CONE_BIG_ORANGE, [])
        )

    @staticmethod
    def from_csv(csv_file_content: str) -> Track:
        # csv file structure:
        # tag, x, y, direction, x_variance, y_variance, xy_covariance

        def extract_xy(row: Sequence[str]):
            return (float(row[1]), float(row[2]))

        rows = csv_file_content.splitlines()[1:]
        row_groups = {k: list(v) for k, v in itertools.groupby(sorted(csv.reader(rows)), key=lambda x: x[0])}        
        cone_groups = {k: [extract_xy(row) for row in rows] for k, rows in row_groups.items() if k != 'car_start'}
        
        car_start_group = row_groups['car_start']
        assert len(car_start_group) == 1, f'Expected exactly one row with tag "car_start", but got {len(car_start_group)}'
        x, y = extract_xy(car_start_group[0])
        yaw = float(car_start_group[0][3])

        return Track(x, y, yaw,
            left_cones=cone_groups.get('blue', []),
            right_cones=cone_groups.get('yellow', []),
            orange_cones=cone_groups.get('orange', []),
            big_orange_cones=cone_groups.get('big_orange', [])
        )

    def as_csv(self) -> str:
        string_buffer = io.StringIO()

        # csv file structure:
        # tag, x, y, direction, x_variance, y_variance, xy_covariance
        writer = csv.writer(string_buffer)

        for x, y in self.left_cones:
            writer.writerow(["yellow", x, y, 0.0, 0.0, 0.0, 0.0])

        for x, y in self.right_cones:
            writer.writerow(["blue", x, y, 0.0, 0.0, 0.0, 0.0])

        for x, y in self.orange_cones:
            writer.writerow(["orange", x, y, 0.0, 0.0, 0.0, 0.0])
        
        for x, y in self.big_orange_cones:
            writer.writerow(["big_orange", x, y, 0.0, 0.0, 0.0, 0.0])

        writer.writerow(['car_start', self.start_x, self.start_y, self.start_yaw, 0.0, 0.0, 0.0])

        return string_buffer.getvalue()

    @staticmethod
    def from_gazebo_sdf(sdf_file_content: str):
        root = xml.etree.ElementTree.fromstring(sdf_file_content)

        left_cones: List[Cone] = []
        right_cones: List[Cone] = []
        orange_cones: List[Cone] = []
        big_orange_cones: List[Cone] = []

        mesh_str_to_cone_list = {
            'blue_cone': left_cones,
            'yellow_cone': right_cones,
            'orange_cone': orange_cones,
            'big_cone': big_orange_cones
        }

        # iterate over all links of the model
        assert len(root[0].findall("include")) != 0

        for i, child in enumerate(root[0].iter("include")):
            pose_raw = child.find("pose")
            assert pose_raw is not None, "No pose found in sdf file"
            assert pose_raw.text is not None, "No pose found in sdf file"
            
            x, y = pose_raw.text.split(" ")[0:2]
            pose = (float(x), float(y))

            name_element = child.find("name")
            assert name_element is not None, f"Name element not present in <inclue> tag number {i}"
            assert name_element.text is not None

            # the name format is <anything>_N
            # for example, blue_cone_0
            mesh_str = name_element.text[:name_element.text.rfind("_")]
            # indentify cones by the name of their mesh
            
            try:
                mesh_str_to_cone_list[mesh_str].append(pose)
            except KeyError:
                print("[track_gen.py] [ERROR] No such object: " + mesh_str)
        
        return Track(0, 0, 0, left_cones, right_cones, orange_cones, big_orange_cones)

    @staticmethod
    def load_from_file(file_name: Union[str, Path]) -> Track:
        file_path = Path(file_name)

        parsers_by_suffix: Dict[str, Callable[[str], Track]] = {
            '.csv': Track.from_csv,
            '.sdf': Track.from_gazebo_sdf,
            '.yaml': lambda x: Track.from_conemap_dict(yaml.safe_load(x)),
        }
        
        try:
            return parsers_by_suffix[file_path.suffix](file_path.read_text())
        except KeyError:
            raise ValueError(f"Attemted to load Track from an unsupported file type: {file_path.suffix}")

    def as_gazebo_sdf(self, model_name: str) -> xml.etree.ElementTree.Element:
        cone_lists_with_names = [(self.right_cones,      "yellow_cone"),
                                 (self.left_cones,       "blue_cone"  ),
                                 (self.big_orange_cones, "big_cone"   ),
                                 (self.orange_cones,     "orange_cone")]

        root = Element("sdf")
        root.set("version", "1.6")
        model = SubElement(root, "model")
        model.set("name", model_name)

        for cone_list, cone_name in cone_lists_with_names:
            for i, (x, y) in enumerate(cone_list):
                include = SubElement(model, "include")
                uri = SubElement(include, "uri")
                uri.text = f'model://{cone_name}'
                pose = SubElement(include, "pose")
                pose.text = f'{x} {y} 0 0 0 0'
                name = SubElement(include, "name")
                name.text = f'{cone_name}_{i}'

        return root

    def as_occupancy_grid(self, resolution: float = 0.05, margin: float = 1.0, cone_radius: float = 0.15) -> Tuple[PIL.Image.Image, float, float]:
        """
        Represents the track as a nav2-compatible map in the occupancy grid format.

        Parameters
        ----------
        resolution : float, optional
            meters per pixel
        margin : float, optional
            minimal distance between any cone and map boundary in meters
        cone_radius : float, optional
            radius of the cones (on the height of LiDAR!) in meters

        Returns
        -------
        tuple[PIL.Image.Image, float, float]
            the image, the origin x coordinate, the origin y coordinate
        """

        cone_coords = itertools.chain(self.left_cones, 
                                    self.right_cones,
                                    self.orange_cones,
                                    self.big_orange_cones)
        
        xs_and_ys: Tuple[List[float], List[float]] = map(list, zip(*cone_coords)) # type: ignore
        xs, ys = xs_and_ys

        xs_min, ys_min = min(xs), min(ys)

        max_x_on_map = 2 * margin + max(xs) - xs_min
        max_y_on_map = 2 * margin + max(ys) - ys_min

        x_pixels = round(max_x_on_map / resolution)
        y_pixels = round(max_y_on_map / resolution)

        if x_pixels * y_pixels > 3e7:
            raise Exception(f"Map is too big to be rendered. Would be {x_pixels * y_pixels / 1e6} million pixels")

        cones_translated = [(x - xs_min + margin, y - ys_min + margin) for x, y in zip(xs, ys)]

        im = PIL.Image.new('RGBA', (x_pixels, y_pixels), 'white')
        draw = PIL.ImageDraw.Draw(im)

        r = cone_radius

        for x, y in cones_translated:
            draw.ellipse(
                [((x - r)/resolution, (y - r)/resolution), 
                 ((x + r)/resolution, (y + r)/resolution)], 
                fill='black')

        im =  PIL.ImageOps.flip(im)
        return im, xs_min - margin, ys_min - margin

    @staticmethod
    def from_conemap_dict(conemap_dict: Dict[str, Any]) -> Track:
        f = lambda cone: (float(cone['position']['x']), float(cone['position']['y'])) # type: ignore

        return Track(0.0, 0.0, 0.0,
            left_cones=[f(cone) for cone in conemap_dict.get('left_cones', [])],
            right_cones=[f(cone) for cone in conemap_dict.get('right_cones', [])],
            orange_cones=[f(cone) for cone in conemap_dict.get('small_orange_cones', [])],
            big_orange_cones=[f(cone) for cone in conemap_dict.get('big_orange_cones', [])],
            frame_id=conemap_dict['header']['frame_id'])
    
    def as_conemap_dict(self) -> Dict[str, Any]:
        mkCone = lambda x, y: {'position': {'x': x, 'y': y, 'z': 0.0}} # type: ignore
        return {
            'header': {'frame_id': self.frame_id},
            'left_cones': [mkCone(x, y) for x, y in self.left_cones],
            'right_cones': [mkCone(x, y) for x, y in self.right_cones],
            'big_orange_cones': [mkCone(x, y) for x, y in self.big_orange_cones],
            'small_orange_cones': [mkCone(x, y) for x, y in self.orange_cones],
        }
    
    def as_conemap_message(self) -> imperial_driverless_interfaces.msg.ConeMap:
        cm_msg = imperial_driverless_interfaces.msg.ConeMap()
        cm_msg.header.frame_id = self.frame_id
        cm_msg.left_cones = list(map(_make_idi_cone, self.left_cones))
        cm_msg.right_cones = list(map(_make_idi_cone, self.right_cones))
        cm_msg.big_orange_cones = list(map(_make_idi_cone, self.big_orange_cones))
        cm_msg.small_orange_cones = list(map(_make_idi_cone, self.orange_cones))
        return cm_msg

    @staticmethod
    def from_conemap_message(msg: imperial_driverless_interfaces.msg.ConeMap) -> Track:
        raise NotImplementedError("This method is not implemented yet")


def _make_idi_cone(xy_tup: Tuple[float, float]) -> imperial_driverless_interfaces.msg.Cone:
    return imperial_driverless_interfaces.msg.Cone(
        position=geometry_msgs.msg.Point(
            x=float(xy_tup[0]),
            y=float(xy_tup[1]),
            z=0.0,
        )
    )
