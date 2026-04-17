import numpy as np


# Named DH presets for known ToRoTRoN joint labels.
# Units: distances in cm, angles in degrees.
TOROTRON_DH_PRESETS = {
    "j1": {"theta0_deg": 0.0, "d": 0.0, "a": 0.0, "alpha_deg": 0.0, "joint_type": "revolute"},
    "joint_1": {"theta0_deg": 0.0, "d": 0.0, "a": 0.0, "alpha_deg": 0.0, "joint_type": "revolute"},
    "shoulder": {"theta0_deg": 0.0, "d": 0.0, "a": 0.0, "alpha_deg": 0.0, "joint_type": "revolute"},
    "elbow": {"theta0_deg": 0.0, "d": 0.0, "a": 0.0, "alpha_deg": 0.0, "joint_type": "revolute"},
    "wrist": {"theta0_deg": 0.0, "d": 0.0, "a": 0.0, "alpha_deg": 0.0, "joint_type": "revolute"},
}


def _norm_name(name):
    return (name or "").strip().lower().replace("-", "_").replace(" ", "_")


def infer_dh_from_offset(joint, ratio):
    t_offset = joint.child_link.t_offset

    px = t_offset[0, 3] / ratio
    py = t_offset[1, 3] / ratio
    pz = t_offset[2, 3] / ratio

    inferred_a = float(np.hypot(px, py))
    inferred_theta0 = float(np.degrees(np.arctan2(py, px))) if inferred_a > 1e-9 else 0.0
    inferred_d = float(pz)
    inferred_alpha = float(np.degrees(np.arctan2(t_offset[2, 1], t_offset[2, 2])))

    return {
        "theta0_deg": inferred_theta0,
        "d": inferred_d,
        "a": inferred_a,
        "alpha_deg": inferred_alpha,
        "joint_type": joint.joint_type,
    }


def resolve_torotron_dh(joint, meta, ratio):
    inferred = infer_dh_from_offset(joint, ratio)

    joint_id = joint.name
    custom_name = meta.get("custom_name", joint_id)

    preset = None
    for key in (_norm_name(custom_name), _norm_name(joint_id), _norm_name(meta.get("joint_id", ""))):
        if key in TOROTRON_DH_PRESETS:
            preset = TOROTRON_DH_PRESETS[key]
            break

    data = dict(inferred)
    if preset:
        data.update(preset)

    # Explicit saved UI DH values always win.
    if "dh_theta" in meta:
        data["theta0_deg"] = float(meta["dh_theta"])
    if "dh_d" in meta:
        data["d"] = float(meta["dh_d"])
    if "dh_a" in meta:
        data["a"] = float(meta["dh_a"])
    if "dh_alpha" in meta:
        data["alpha_deg"] = float(meta["dh_alpha"])
    if "joint_type" in meta:
        data["joint_type"] = str(meta["joint_type"]).lower()
    else:
        data["joint_type"] = str(data.get("joint_type", "revolute")).lower()

    return data


def compute_joint_matrix(theta_deg, d, a, alpha_deg, joint_type="revolute", q_value=0.0):
    joint_type = (joint_type or "revolute").lower()
    if joint_type == "prismatic":
        theta_eff_deg = theta_deg
        d_eff = d + q_value
    else:
        theta_eff_deg = theta_deg + q_value
        d_eff = d

    theta = np.radians(theta_eff_deg)
    alpha = np.radians(alpha_deg)

    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d_eff],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def compute_forward_kinematics(dh_rows):
    cumulative = np.eye(4)
    results = []
    for row in dh_rows:
        local = compute_joint_matrix(
            theta_deg=row["theta0_deg"],
            d=row["d"],
            a=row["a"],
            alpha_deg=row["alpha_deg"],
            joint_type=row["joint_type"],
            q_value=row["q_value"],
        )
        cumulative = cumulative @ local
        results.append({
            "title": row["title"],
            "joint_name": row["joint_name"],
            "local": local,
            "cumulative": cumulative.copy(),
            "joint_type": row["joint_type"],
            "theta0_deg": row["theta0_deg"],
            "d": row["d"],
            "a": row["a"],
            "alpha_deg": row["alpha_deg"],
            "q_value": row["q_value"],
        })
    return results
