#!/usr/bin/env python3
"""Build experiment003 mass/geometry from the documented B-D prototype BOM."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
NOMINAL_MASS_G = 12338.7
REFERENCE_MASS_KG = 13.4946
WHEELBASE_M = 1.172
ARM_XY_M = WHEELBASE_M / 2 / math.sqrt(2)
MOTOR_POSITIONS = [
    [ARM_XY_M, ARM_XY_M, 0], [-ARM_XY_M, -ARM_XY_M, 0],
    [ARM_XY_M, -ARM_XY_M, 0], [-ARM_XY_M, ARM_XY_M, 0],
]
DOCS_COMMIT = "83dfb0be7130008cc49d900d6633d6adef8740dd"
DOCS_URL = f"https://github.com/m8k8/drone-docs/blob/{DOCS_COMMIT}/hardware/prototype-01/README.md"


def item(key, name, stage, quantity, unit_g, evidence, position, dimensions=None, source=None):
    """All positions/dimensions are provisional mass-distribution assumptions."""
    return dict(id=key, name=name, stage=stage, quantity=quantity,
                unit_mass_g=unit_g, total_mass_g=round(quantity * unit_g, 6),
                mass_evidence=evidence, source=source or DOCS_URL,
                center_m=position, box_dimensions_m=dimensions,
                placement_status="assumed, not a measured or verified installation")


def make_bom():
    rows = [
        item("fc", "Pixhawk 6C Plastic SKU11054", "B", 1, 34.6, "published", [0, 0, -.02], [.0848, .044, .0124], "https://holybro.com/products/pixhawk-6c"),
        item("gps", "M10 IST8310 Standard10pin SKU12040", "B", 1, 32, "published_scope_conditional", [0, 0, -.12], None, "https://holybro.com/products/m10-gps"),
        item("frame", "G06 frame, legs/supports retained; tank omitted", "C", 1, 5400, "published_scope_conditional", [0, 0, .04], [.40, .32, .20], "https://www.effort-tech.com/en/g06s"),
        item("propulsion", "X6 Plus G2 including MFP24x8 props and stock cables", "C", 4, 748, "published", MOTOR_POSITIONS, None, "https://www.hobbywing.com/en/uploads/file/20260104/51c225f38bf68c65072228943ae15f20.pdf"),
        item("battery", "ETop 6S 10Ah25C AS150/XT150; two packs in series", "C", 2, 1290, "seller_published", [[0, -.0355, .18], [0, .0355, .18]], [.170, .061, .058], "https://rc-matrix.com/SHOP/4-6S-10000-25CS-3.html"),
        item("pm08", "PM08-CAN tinned wires SKU15029 including stock cables", "C", 1, 185, "published_scope_conditional", [-.08, 0, .03], [.045, .041, .026], "https://holybro.com/products/dronecan-pm08-power-module-14s-200a"),
        item("pdb", "MATEKSYS FCHUB-12S V2", "C", 1, 21, "published", [-.02, 0, .03], [.055, .050, .006], "https://www.mateksys.com/?portfolio=fchub-12s"),
        item("receiver", "GR01 receiver, conditional Japanese G12 bundle", "C", 1, 37, "published_scope_conditional", [.05, 0, 0]),
        item("remote_id", "Futaba FRID-1", "C", 1, 9, "published", [-.10, 0, 0], None, "https://www.rc.futaba.co.jp/products/detail/I00000343"),
        item("sd", "Transcend TS32GUSD350V32GB, card only", "C", 1, .4, "published", [0, 0, -.02]),
        item("rangefinder", "Benewake TF02-Pro including80cm cable", "D", 1, 50, "published", [.08, 0, .16], None, "https://ai.benewake.com/en/resources/technical-documents/tf02-pro-datasheet/"),
        item("camera", "3-in-1 Ethernet camera plus supplied cables", "C", 1, 150, "design_allowance", [.18, 0, .03]),
        item("antennas", "G12 antennas/feeders, one conditional bundle; physical antenna count unknown", "C", 1, 20, "design_allowance", [0, 0, 0]),
        item("main_harness", "Additional8AWG harness,4mating contacts, insulation", "C", 1, 120, "design_allowance", [0, 0, .10]),
        item("aux_harness", "Additional auxiliary/signal wiring incl M10 cable if excluded", "C", 1, 40, "design_allowance", [0, 0, .02]),
        item("camera_fuse", "SCHURTER8020.5014/0031.1804 and mounting", "C", 1, 20, "design_allowance", [.10, 0, .02]),
        item("battery_tray", "EPXA220x160x3mm battery tray", "C", 1, 200.6, "material_calculation", [0, 0, .215], [.220, .160, .003]),
        item("tray_support", "EPXA100x20x3mm support/stopper plates", "C", 4, 11.4, "material_calculation", [[-.07, -.06, .15], [-.07, .06, .15], [.07, -.06, .15], [.07, .06, .15]], [.100, .020, .003]),
        item("straps", "ONE-WRAP25.4x450mm straps", "C", 4, 10, "design_allowance", [[-.05, -.0355, .18], [.05, -.0355, .18], [-.05, .0355, .18], [.05, .0355, .18]]),
        item("battery_pad", "EPTSEALER6800 pads and end cushioning", "C", 1, 30, "design_allowance", [0, 0, .209], [.170, .132, .002]),
        item("gps_mast", "M10 supplied Fixed Carbon Fiber GPS mount", "C", 1, 20, "design_allowance", [0, 0, -.06], [.012, .012, .12]),
        item("avionics_tray", "EPXA140x120x3mm FC/receiver base", "C", 1, 95.8, "material_calculation", [0, 0, -.008], [.140, .120, .003]),
        item("fc_tape", "3M VHB4941 four20x20x1.1mm pieces", "C", 1, 1.3, "material_calculation", [0, 0, -.012]),
        item("rf_mount", "EPXA80x50x3mm downward rangefinder mount", "D", 1, 22.8, "material_calculation", [.08, 0, .145], [.080, .050, .003]),
        item("camera_mount", "A5052 bracket from100x40x2mm blank", "C", 1, 21.4, "material_calculation", [.15, 0, .03]),
        item("rid_case", "Takachi SW-40B", "C", 1, 10, "published", [-.10, 0, 0], [.040, .030, .020], "https://www.takachi-el.co.jp/products/SW"),
        item("rid_pad", "Takachi UR-40", "C", 1, .8, "published", [-.10, 0, 0], [.023, .033, .005], "https://www.takachi-el.co.jp/products/UR"),
        item("fasteners", "Additional screws,nuts,standoffs,washers", "C", 1, 55, "design_allowance", [0, 0, .05]),
        item("power_tray", "EPXA150x90x3mm separate power base", "C", 1, 77, "material_calculation", [-.05, 0, .05], [.150, .090, .003]),
        item("pdb_guard", "EPXA80x60x3mm open-sided PDB guard", "C", 1, 27.4, "material_calculation", [-.02, 0, .01], [.080, .060, .003]),
    ]
    assert math.isclose(sum(r["total_mass_g"] for r in rows), NOMINAL_MASS_G, abs_tol=1e-8)
    return {
        "source_document": DOCS_URL, "source_docs_commit": DOCS_COMMIT,
        "scope": "prototype stagesB-D, no tank/water/sprayer/RTK/onboard PC",
        "mass_unit": "g", "coordinates": "x forward,y right,z down; origin at motor-plane geometric center",
        "layout_status": "Every installation coordinate is assumed. Hardware mounting and CG have not been measured.",
        "rows": rows, "nominal_total_mass_g": NOMINAL_MASS_G,
        "not_additionally_counted": ["MFP24x8 propellers and X6 stock cables already in748g/rotor", "PM08 stock cables already in185g", "TF02 supplied80cm cable already in50g", "Ground transmitter/charger/PC/tools/spares", "Tank mass not subtracted from conditional5400g frame figure"],
    }


def mass_elements(bom, mass_delta_kg):
    elements = []
    for row in bom["rows"]:
        if row["id"] == "antennas":
            # Two equal mass lumps approximate one bundle; this is not a claim
            # that the domestic G12 kit contains two antennas.
            for i,y in enumerate([-.12,.12]):
                elements.append(dict(id=f"antenna-bundle-lump-{i+1}", mass_kg=.01,
                                     center_m=[0,y,0], dimensions_m=None))
            continue
        if row["id"] == "frame":
            # Only a distribution hypothesis: 2.4kg in four arm/support lumps,
            # remaining3.0kg central box. Not inferred from a CAD model.
            elements.append(dict(id="frame-central", mass_kg=3.0 + mass_delta_kg,
                                 center_m=row["center_m"], dimensions_m=row["box_dimensions_m"]))
            for i, p in enumerate(MOTOR_POSITIONS):
                elements.append(dict(id=f"frame-arm-{i+1}", mass_kg=.6,
                                     center_m=[p[0]/2, p[1]/2, 0], dimensions_m=None))
            continue
        positions = row["center_m"]
        if not isinstance(positions[0], list):
            assert row["quantity"] == 1
            positions = [positions]
        assert len(positions) == row["quantity"]
        for i, position in enumerate(positions):
            elements.append(dict(id=f"{row['id']}-{i+1}", mass_kg=row["unit_mass_g"]/1000,
                                 center_m=position, dimensions_m=row["box_dimensions_m"]))
    return elements


def properties(elements):
    mass = sum(e["mass_kg"] for e in elements)
    cg = [sum(e["mass_kg"]*e["center_m"][j] for e in elements)/mass for j in range(3)]
    tensor = [[0.0]*3 for _ in range(3)]
    for e in elements:
        m = e["mass_kg"]
        x,y,z = [p-c for p,c in zip(e["center_m"],cg)]
        intrinsic = [0,0,0]
        if e["dimensions_m"]:
            a,b,c = e["dimensions_m"]
            intrinsic = [m*(b*b+c*c)/12, m*(a*a+c*c)/12, m*(a*a+b*b)/12]
        for i,v in enumerate([m*(y*y+z*z),m*(x*x+z*z),m*(x*x+y*y)]):
            tensor[i][i] += v + intrinsic[i]
        for i,j,v in [(0,1,-m*x*y),(0,2,-m*x*z),(1,2,-m*y*z)]:
            tensor[i][j] += v
            tensor[j][i] += v
    diagonal = [tensor[i][i] for i in range(3)]
    assert all(x > 0 for x in diagonal)
    return dict(mass_kg=round(mass,7), cg_from_motor_plane_m=cg,
                inertia_tensor_about_cg_kg_m2=tensor,
                native_diagonal_inertia_kg_m2=diagonal,
                native_motor_positions_from_cg_m=[[p[i]-cg[i] for i in range(3)] for p in MOTOR_POSITIONS])


def render_json(value):
    return json.dumps(value, indent=2, ensure_ascii=False) + "\n"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    propulsion_path = HERE/"propulsion.json"
    propulsion = json.loads(propulsion_path.read_text())
    base = propulsion["model_parameters"]
    geometry_fields = {"mass", "moment_inertia", *(f"motor{i}_position" for i in range(1,5))}
    assert not geometry_fields.intersection(base), "Propulsion input must not contain another aircraft model"
    assert base["referenceMass"] == REFERENCE_MASS_KG
    bom = make_bom()
    outputs = {"components.json": render_json(bom)}
    cases, physical = [], []
    fixed_fields = set(base)
    for label, delta in [("nominal",0.0),("mass-low",-.5),("mass-high",.5)]:
        elements = mass_elements(bom, delta)
        props = properties(elements)
        model = {"mass": props["mass_kg"], **copy.deepcopy(base)}
        for i, p in enumerate(props["native_motor_positions_from_cg_m"],1):
            model[f"motor{i}_position"] = p
        model["moment_inertia"] = props["native_diagonal_inertia_kg_m2"]
        assert all(model[k] == base[k] for k in fixed_fields), "Propulsion/drag calibration changed"
        assert math.isclose(model["mass"], NOMINAL_MASS_G/1000 + delta, abs_tol=1e-8)
        path = f"models/{label}.json"
        outputs[path] = render_json(model)
        cases.append(dict(label=label, model=path, gross_mass_kg=model["mass"], mass_delta_kg=delta,
                          inertia_kg_m2=model["moment_inertia"], cg_from_motor_plane_m=props["cg_from_motor_plane_m"],
                          sensitivity_status="nominal design estimate" if delta==0 else "assumed sensitivity case, not a bound or real BOM"))
        physical.append(dict(label=label, **props, mass_elements=elements))
    outputs["mass-properties.json"] = render_json({
        "frame_distribution_assumption":"4x0.6kg arm/support lumps at half motor radius;3.0kg central400x320x200mm box centered40mm belowmotorplane",
        "battery_distribution_assumption":"2x1.29kg packs,170x61x58mm each,centersy=+-35.5mm,z=180mm belowmotorplane;10mm gap",
        "sensitivity_assumption":"Apply+-0.5kg only to assumed centralframe lump; not measured mass uncertainty bounds",
        "native_inertia_limit":"SIM_Frame uses only diagonal moments; products of inertia and general rigid-body Euler cross terms are not implemented",
        "native_contact_limit":"SIM_Multicopter frame_height remains0.1m; no mesh,real landinggear/tankbay collisiongeometry or structural flex",
        "sensor_geometry_limit":"Virtual rangefinder and primary sensors colocated with modeledCG; hardware mount coordinates in BOM affect inertia only",
        "cases":physical,
    })
    outputs["manifest.json"] = render_json({
        "model_status":"B-D prototype provisional physics and sensor model, not a validated real-aircraft digital twin",
        "upstream_commit":propulsion["upstream_commit"], "frame":"X", "defaults":"defaults.parm",
        "empty_mass_kg":round(NOMINAL_MASS_G/1000,7), "nominal_mass_kg":round(NOMINAL_MASS_G/1000,7),
        "reference_mass_kg":REFERENCE_MASS_KG, "source_docs_commit":DOCS_COMMIT,
        "source_bom":DOCS_URL, "components":"components.json", "mass_properties":"mass-properties.json",
        "propulsion_parameters":"propulsion.json", "propulsion_parameters_sha256":sha256(propulsion_path),
        "calibration":"calibration.json", "reference_mass_patch":"reference-mass.patch",
        "patch_sha256":propulsion["patch_sha256"], "patch_files":propulsion["patch_files"],
        "launch_args":["--serial5=sim:benewake_tf03"],
        "rangefinder":{"product_intent":"TF02-Pro UART", "native_serial_emulator":"benewake_tf03", "driver_type":27,
                       "reading_rate_hz":5,"virtual_position_from_cg_m":[0,0,0],"distance_min_m":.1,"distance_max_m":13.5,
                       "noise_setting_m":.02,"noise_status":"assumed native uniform random addition, not identified TF02-Pro noise"},
        "geometry_assumption":"1172mm squareX approximation; motor leverarms translated to assumedCG",
        "propulsion_calibration_unchanged":True, "cases":cases,
        "functional_coverage":{
            "ArduCopter":"actual compiled controller running onhost; Pixhawk CPU/powerhardware notemulated",
            "M10":"generic SITL GPS/compass; not an identified M10 error model or RTK",
            "TF02-Pro":"native compatible UART packet path and driver;5Hz emulator,notreal100Hz device/optics",
            "G12":"RC/MAVLink control interface only; radio propagation,packet delay/loss not modeled by nominalconfig",
            "FPV":"camera mass/assumed placement only; no images,networkvideoor latency",
            "RemoteID":"mass/assumed placement only; no RFbroadcast",
            "power_distribution":"mass/assumed placement only; no connector thermal/inrush/fuse behavior",
            "microSD":"mass; flight logs use host filesystem rather than physicalcardhardware",
        },
        "limitations":[
            "Frame5400g scope unresolved; additionalweights include allowances; nominal mass not measurementor bound",
            "All installationpositions are assumptions; geometry,CG,inertia andbatterytray strength unmeasured",
            "Fixed linear power/thrust fit and assumed0.06ohm packresistance; no endurancevalidation",
            "Native batterycurrent sums motors only; camera,radio,FC,sensorsandDC-DC consumption not added",
            "Genericgroundcontact0.1m,diagonal inertiaonly; no structurecollisionor flexibility",
            "No tank,water,sprayer,RTK,AIimageprocessingor obstacleavoidance",
        ],
    })
    for relative, content in outputs.items():
        path = HERE/relative
        if args.check:
            if not path.exists() or path.read_text() != content:
                raise SystemExit(f"Generated file mismatch: {path}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    print(f"{'Verified' if args.check else 'Generated'} prototypeB-D models: nominal{NOMINAL_MASS_G/1000:.4f}kg, mass-low/high+-0.5kg")
    print("Propulsion anddrag reference unchanged; CG/inertia from explicit assumed component placements.")


if __name__ == "__main__":
    main()
