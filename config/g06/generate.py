#!/usr/bin/env python3
"""Generate provisional G06 models; no running vehicle or upstream file is changed."""
import argparse
import difflib
import hashlib
import json
import math
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
UPSTREAM = ROOT / "vendor/ardupilot"
UPSTREAM_COMMIT = "fb773567896014da007bca58358704d4a0d8516f"
BASE_MASS = 13.4946
BENCH_VOLTAGE = 46.0
FULL_VOLTAGE = 50.4
GRAVITY = 9.80665
SEA_LEVEL_DENSITY = 1.225
WHEELBASE = 1.172
ROTOR_DIAMETER = 0.6096
MOTOR_UNIT_MASS = 0.748
BENCH_URL = "https://www.hobbywing.com/en/products/x6-plus-g2"
BENCH_PDF = "https://www.hobbywing.com/uploads/file/20251212/be8db6e8539e5b63271859811f66214a.pdf"
FRAME_URL = "https://c.effort-tech.com/yft/service/t9pfu2ax8fcwpyk2bxw2.pdf"

# Manufacturer test data: throttle fraction, kgf/rotor, A/rotor, input W/rotor.
DATA = [
    (.33, 1.736, 3.0, 137.7), (.35, 2.017, 3.6, 167.2),
    (.37, 2.318, 4.4, 201.7), (.39, 2.634, 5.3, 241.8),
    (.42, 3.131, 6.7, 310.4), (.45, 3.646, 8.4, 388.7),
    (.48, 4.174, 10.4, 476.5), (.51, 4.712, 12.4, 572.3),
    (.54, 5.256, 14.7, 676.2), (.57, 5.807, 17.1, 787.6),
    (.60, 6.364, 19.7, 907.1), (.63, 6.927, 22.5, 1035.0),
    (.66, 7.495, 25.5, 1171.4), (.69, 8.068, 28.6, 1316.8),
    (.72, 8.644, 32.0, 1470.6), (.75, 9.219, 35.5, 1632.5),
    (.78, 9.786, 39.2, 1800.5), (.81, 10.337, 42.9, 1971.2),
    (.84, 10.861, 46.6, 2140.4), (.87, 11.342, 50.1, 2301.3),
    (.90, 11.762, 53.2, 2445.1), (1.0, 12.526, 58.0, 2663.7),
]


def json_text(value):
    return json.dumps(value, indent=2, ensure_ascii=False) + "\n"


def fit_thrust():
    # Fit measured 33..51% operating band while preserving measured max thrust.
    # The effective zero intercept is a numerical fit, not an ESC specification.
    best = None
    for step in range(-300, 321):
        offset = step / 1000
        command = [max(0, min(1, (t-offset)/(1-offset))) for t, *_ in DATA[:8]]
        b = [c*c-c for c in command]
        expo = max(0, min(1, sum(v*(row[1]/DATA[-1][1]-c) for v,c,row in zip(b,command,DATA[:8])) / sum(v*v for v in b)))
        error = sum((DATA[-1][1]*(c+expo*v)-row[1])**2 for c,v,row in zip(command,b,DATA[:8])) / 8
        candidate = (error, offset, expo)
        if best is None or candidate < best:
            best = candidate
    return best[1], best[2]


def interpolate(thrust_kgf, column):
    for a,b in zip(DATA, DATA[1:]):
        if a[1] <= thrust_kgf <= b[1]:
            ratio = (thrust_kgf-a[1])/(b[1]-a[1])
            return a[column] + ratio*(b[column]-a[column])
    raise ValueError("Requested interpolation outside manufacturer data")


def build_patch():
    transformations = {
        "libraries/SITL/SIM_Frame.h": [
            ("        float mass = 3.0;\n", "        float mass = 3.0;\n\n        // Optional fixed propulsion/drag calibration mass; zero preserves legacy behavior.\n        float referenceMass = 0.0;\n"),
        ],
        "libraries/SITL/SIM_Frame.cpp": [
            ("        FRAME_VAR(mass),\n", "        FRAME_VAR(mass),\n        FRAME_VAR(referenceMass),\n"),
            ("    mass = model.mass;\n\n    const float drag_force = model.mass *", "    mass = model.mass;\n    const float reference_mass = model.referenceMass > 0 ? model.referenceMass : model.mass;\n    if (!is_positive(mass) || !is_positive(reference_mass) || model.referenceMass < 0) {\n        AP_HAL::panic(\"Invalid frame mass/referenceMass\");\n    }\n    ::printf(\"G06 frame mass=%.6f referenceMass=%.6f\\n\", (double)mass, (double)reference_mass);\n    ::fflush(stdout);\n\n    const float drag_force = reference_mass *"),
            ("    const float ref_thrust = model.mass *", "    const float ref_thrust = reference_mass *"),
            ("    float hover_thrust = mass * GRAVITY_MSS;", "    float hover_thrust = reference_mass * GRAVITY_MSS;"),
        ],
    }
    chunks, hashes = [], {}
    for path, replacements in transformations.items():
        before = subprocess.check_output(["git", "-C", str(UPSTREAM), "show", f"{UPSTREAM_COMMIT}:{path}"], text=True)
        after = before
        for old,new in replacements:
            if after.count(old) != 1:
                raise RuntimeError(f"Patch anchor not unique: {path}: {old}")
            after = after.replace(old,new,1)
        chunks.append(f"diff --git a/{path} b/{path}\n")
        chunks.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile=f"a/{path}", tofile=f"b/{path}"))
        hashes[path] = {"original_sha256": hashlib.sha256(before.encode()).hexdigest(), "patched_sha256": hashlib.sha256(after.encode()).hexdigest()}
    return "".join(chunks), hashes


def propulsion_constants(model):
    thrust = model["referenceMass"] * GRAVITY
    power = model["refCurrent"] * model["refVoltage"]
    outflow = 2*power/thrust
    return {
        "power_factor_w_per_n": power/thrust,
        "effective_prop_area_m2": thrust/(.5*SEA_LEVEL_DENSITY*outflow*outflow)/4,
        "max_outflow_velocity_m_s": outflow/math.sqrt(model["hoverThrOut"]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="Verify checked-in generated files, without writing")
    args = parser.parse_args()
    offset, expo = fit_thrust()
    power_ref = 4*interpolate(BASE_MASS/4, 3)
    max_thrust_full_kgf = DATA[-1][1] * (FULL_VOLTAGE/BENCH_VOLTAGE)**2
    radius = WHEELBASE/2
    arm_xy = radius/math.sqrt(2)
    # Four propulsion units at motor centers; remaining mass in an assumed box.
    center_mass = BASE_MASS-4*MOTOR_UNIT_MASS
    center_length, center_width, center_height = .50, .35, .30
    base_ix = 4*MOTOR_UNIT_MASS*arm_xy**2 + center_mass*(center_width**2+center_height**2)/12
    base_iy = 4*MOTOR_UNIT_MASS*arm_xy**2 + center_mass*(center_length**2+center_height**2)/12
    base_iz = 8*MOTOR_UNIT_MASS*arm_xy**2 + center_mass*(center_length**2+center_width**2)/12
    base = {
        "mass": BASE_MASS, "referenceMass": BASE_MASS,
        "diagonal_size": radius,
        "refSpd": 10.0, "refAngle": 10.0,
        "refVoltage": BENCH_VOLTAGE, "refCurrent": power_ref/BENCH_VOLTAGE,
        "refAlt": 0.0, "refTempC": 25.0, "refBatRes": .06,
        "maxVoltage": FULL_VOLTAGE, "battCapacityAh": 10.0,
        "hoverThrOut": BASE_MASS/(4*max_thrust_full_kgf),
        "propExpo": expo, "refRotRate": 120.0,
        "pwmMin": 1050.0, "pwmMax": 1950.0,
        "spin_min": offset, "spin_max": 1.0, "slew_max": 20.0,
        "disc_area": 4*math.pi*(ROTOR_DIAMETER/2)**2,
        "mdrag_coef": .2, "num_motors": 4.0,
        "motor1_position": [arm_xy,arm_xy,0.0],
        "motor2_position": [-arm_xy,-arm_xy,0.0],
        "motor3_position": [arm_xy,-arm_xy,0.0],
        "motor4_position": [-arm_xy,arm_xy,0.0],
    }
    outputs, cases = {}, []
    constants = None
    for water in (0,1,3,5):
        model = dict(base)
        model["mass"] = round(BASE_MASS+water, 6)
        # Centered cylindrical payload, r=0.15m,h=0.22m. No slosh or moving CG.
        ixy = water*(3*.15**2+.22**2)/12
        iz = water*.15**2/2
        model["moment_inertia"] = [base_ix+ixy, base_iy+ixy, base_iz+iz]
        path = f"models/payload-{water}l.json"
        outputs[path] = json_text(model)
        case_constants = propulsion_constants(model)
        if constants is None:
            constants = case_constants
        assert case_constants == constants, "Propulsion must stay identical as payload varies"
        cases.append({"label":f"payload-{water}l", "model":path, "water_l":water, "gross_mass_kg":model["mass"], "inertia_kg_m2":model["moment_inertia"]})
    residuals = []
    factor = constants["power_factor_w_per_n"]
    for throttle,thrust,amps,power in DATA:
        command = max(0,min(1,(throttle-offset)/(1-offset)))
        predicted_thrust = DATA[-1][1]*((1-expo)*command+expo*command*command)
        predicted_current = factor*predicted_thrust*GRAVITY/BENCH_VOLTAGE
        residuals.append({"throttle_pct":throttle*100, "measured_thrust_kgf":thrust, "model_thrust_kgf":predicted_thrust, "thrust_error_pct":100*(predicted_thrust/thrust-1), "measured_current_a":amps, "model_current_a":predicted_current, "current_error_pct":100*(predicted_current/amps-1), "measured_input_power_w":power})
    calibration = {
        "source":BENCH_URL, "source_pdf":BENCH_PDF,
        "conditions":"Manufacturer 46 V, sea level, 25 C; static bench data. SITL fit evaluated at its sea-level density.",
        "fit_band_throttle_pct":[33,51], "effective_zero_throttle_fraction":offset, "prop_expo":expo,
        "thrust_fit_band_max_abs_error_pct":max(abs(r["thrust_error_pct"]) for r in residuals[:8]),
        "thrust_full_table_max_abs_error_pct":max(abs(r["thrust_error_pct"]) for r in residuals),
        "current_fit_band_max_abs_error_pct":max(abs(r["current_error_pct"]) for r in residuals[:8]),
        "current_full_table_max_abs_error_pct":max(abs(r["current_error_pct"]) for r in residuals),
        "propulsion_constants_equal_across_cases":True, "propulsion_constants":constants,
        "rows":residuals,
    }
    outputs["calibration.json"] = json_text(calibration)
    patch, hashes = build_patch()
    outputs["reference-mass.patch"] = patch
    outputs["manifest.json"] = json_text({
        "model_status":"provisional sensitivity model, not a validated G06 digital twin",
        "upstream_commit":UPSTREAM_COMMIT, "frame":"X", "defaults":"defaults.parm",
        "empty_mass_kg":BASE_MASS, "empty_mass_assumption":"Conditional published partial sum12.4946kg including separately counted tank, plus explicit1.0kg allowance for unresolved parts. This is a scenario, not measured or guaranteed upper/lower bound.",
        "reference_mass_kg":BASE_MASS, "water_density_kg_l":1.0,
        "sources":{"propulsion":BENCH_URL,"propulsion_table":BENCH_PDF,"frame_catalog":FRAME_URL},
        "patch_files":hashes,"patch_sha256":hashlib.sha256(patch.encode()).hexdigest(),
        "geometry_assumption":"Square X approximating official1172mm wheelbase, not exact rectangular G06 motor locations.",
        "inertia_assumption":{"propulsion_unit_mass_kg":MOTOR_UNIT_MASS,"central_box_m":[center_length,center_width,center_height],"centered_payload_cylinder_radius_m":.15,"centered_payload_cylinder_height_m":.22},
        "battery_assumption":{"cells_series":12,"capacity_ah":10,"initial_voltage_v":FULL_VOLTAGE,"pack_resistance_ohm":.06,"resistance_is_measured":False},
        "cases":cases,
        "limitations":["Tank/frame weight overlap unresolved; no actual empty mass", "Estimated inertia, zero CG offset, no liquid slosh or structural flexibility", "Fitted static thrust curve and linear power-per-thrust model, not manufacturer motor dynamics", "Voltage scaling and battery resistance are assumptions, not an endurance validation", "No pump flow,RTK correction,camera,rangefinder,obstacles or failures modeled"],
    })
    for relative, content in outputs.items():
        path = HERE/relative
        if args.check:
            if not path.exists() or path.read_text() != content:
                raise SystemExit(f"Generated file mismatch: {path}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    print(f"{'Verified' if args.check else 'Generated'} 4 models; fixed propulsion across payloads.")
    print(f"Thrust fit error33..51%: {calibration['thrust_fit_band_max_abs_error_pct']:.3f}%; full range: {calibration['thrust_full_table_max_abs_error_pct']:.3f}%")
    print(f"Current error33..51%: {calibration['current_fit_band_max_abs_error_pct']:.3f}%; full range: {calibration['current_full_table_max_abs_error_pct']:.3f}%")


if __name__ == "__main__":
    main()
