"""Display geometry for the flat field scenario."""


CAMERA_POSE = "-31 -25 34 0 0.592 0.911"


def add_crop_rows(link, add_visual):
    # Gazebo uses east for x and north for y. Leave headlands at y=6 and y=30.
    add_visual(link, "cultivated_soil", (0, 18, 0.005, 0, 0, 0), "box",
               {"size": "24 28 0.01"}, "0.35 0.24 0.16 1")
    for row, east in enumerate(range(-10, 11, 2)):
        add_visual(link, f"furrow_{row}", (east, 18, 0.05, 0, 0, 0), "box",
                   {"size": "0.95 20 0.1"}, "0.44 0.31 0.20 1")
        for plant, north in enumerate(range(8, 29, 2)):
            color = "0.24 0.43 0.13 1" if (row + plant) % 2 else "0.32 0.51 0.17 1"
            add_visual(link, f"crop_{row}_{plant}", (east, north, 0.19, 0, 0, 0), "ellipsoid",
                       {"radii": "0.33 0.75 0.16"}, color)
