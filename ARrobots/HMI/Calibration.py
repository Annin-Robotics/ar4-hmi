def apply_calibration(TMPDATA, CAL):
    """
    Load values from TMPDATA into CAL.
    If CAL[key] is a Tkinter variable (IntVar, BooleanVar, etc.), use .set()
    Otherwise, direct assignment.
    """
    for key, value in TMPDATA.items():
        if key in CAL:
            # Check if it's a Tkinter variable (has a 'set' method)
            if hasattr(CAL[key], 'set') and callable(CAL[key].set):
                CAL[key].set(value)
            else:
                CAL[key] = value
        else:
            # Key doesn't exist in CAL, just add it
            CAL[key] = value