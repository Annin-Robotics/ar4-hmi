import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.NOTSET) # Inherit level from Parent

import os
import pickle
import json
from tkinter import Listbox, END, Tk
import ttkbootstrap as ttk_bootstrap
from typing import Union

def save_calibration(calibration_data: dict, calibration_file: str="ARconfig.json") -> bool:
    ''' Save calibration data to JSON file '''

    # Recast tk vars appropriately as they are not JSON serializable
    data_to_save = {}
    for key, value in calibration_data.items():
        # Check if it's a Tkinter variable (has a 'get' method)
        if hasattr(value, 'get') and callable(value.get):
            try:
                data_to_save[key] = value.get()
            except Exception as e:
                # Variable not initialized, skip or use default
                logger.warning(f"Skipping uninitialized variable {key}: {e}")
                data_to_save[key] = None  # or 0, or skip this key entirely
        else:
            data_to_save[key] = value
    
    # Ensure the data is json serializable before writing
    try:
        json_data = json.dumps(data_to_save, indent=4)
    except Exception as e:
        logger.error(f"Error saving json file as the data is not serializable: {e}")
        return False
    try:
        with open(calibration_file, 'w') as f:
            f.write(json_data)
        return True
    except Exception as e:
        logger.error(f"Error saving calibration: {e}")
        return False
    
def load_calibration(calibration_file: str='ARconfig.json', defaults_file='defaults.json') -> dict | None:
    ''' Load calibration data from JSON or convert from old pickle format '''
    try:
        if os.path.exists(calibration_file):
            logger.debug("JSON config file found, loading")
            with open(calibration_file, 'r') as f:
                calibration_data = json.load(f)
        elif os.path.exists('ARbot.cal'):
            logger.debug("No JSON config file found, attempting to load ARbot.cal")
            calibration_data = convert_calibration()
        else:
            logger.info("No configuration file found - Loading default settings")
            if os.path.exists(defaults_file):
                logger.debug("default JSON config file found, loading")
                with open(defaults_file, 'r') as f:
                    calibration_data = json.load(f)
                save_calibration(calibration_data)
        if calibration_data != None:
            return calibration_data
        else:
            logger.error("Error loading calibration data")
            return None
    except Exception as e:
        logger.error(f"Unrecoverable error reading config file.  Maae sure it isn't corrupted.\n{e}")
        exit()
    
def convert_calibration()-> dict | None:
    ''' Convert old ARbot.cal pickle file to new dictionary format and save as ARconfig.json '''
    CAL = {}
    temp = Tk()
    nb = ttk_bootstrap.Notebook(temp)
    tab0 = ttk_bootstrap.Frame(nb)
    calibration = Listbox(tab0,height=60)

    if os.path.exists('ARbot.cal'):
        logger.info("Converting ARbot.cal to JSON format")
        try:
            pickle_data = pickle.load(open("ARbot.cal", "rb"))
        except Exception as e:
            logger.error(f"Error loading calibration: {e}")
            return None
    else:
        logger.error("No ARbot.cal file found, cannot convert")
        return None
    
    try:
        # Populate the traditional Calibration data
        for item in pickle_data:
            calibration.insert(END,item)

        # Map the Listbox items to the CAL dictionary
        CAL['J1AngCur'] = calibration.get("0")
        CAL['J2AngCur'] = calibration.get("1")
        CAL['J3AngCur'] = calibration.get("2")
        CAL['J4AngCur'] = calibration.get("3")
        CAL['J5AngCur'] = calibration.get("4")
        CAL['J6AngCur'] = calibration.get("5")
        CAL['XcurPos'] = calibration.get("6")
        CAL['YcurPos'] = calibration.get("7")
        CAL['ZcurPos'] = calibration.get("8")
        CAL['RxcurPos'] = calibration.get("9")
        CAL['RycurPos'] = calibration.get("10")
        CAL['RzcurPos'] = calibration.get("11")
        CAL['comPort'] = calibration.get("12")
        CAL['Prog'] = calibration.get("13")
        CAL['Servo0on'] = calibration.get("14")
        CAL['Servo0off'] = calibration.get("15")
        CAL['Servo1on'] = calibration.get("16")
        CAL['Servo1off'] = calibration.get("17")
        CAL['DO1on'] = calibration.get("18")
        CAL['DO1off'] = calibration.get("19")
        CAL['DO2on'] = calibration.get("20")
        CAL['DO2off'] = calibration.get("21")
        CAL['TFx'] = calibration.get("22")
        CAL['TFy'] = calibration.get("23")
        CAL['TFz'] = calibration.get("24")
        CAL['TFrx'] = calibration.get("25")
        CAL['TFry'] = calibration.get("26")
        CAL['TFrz'] = calibration.get("27")
        CAL['J7PosCur'] = calibration.get("28")
        CAL['J8PosCur'] = calibration.get("29")
        CAL['J9PosCur'] = calibration.get("30")
        CAL['VisFileLoc'] = calibration.get("31")
        CAL['VisProg'] = calibration.get("32")
        CAL['VisOrigXpix'] = calibration.get("33")
        CAL['VisOrigXmm'] = calibration.get("34")
        CAL['VisOrigYpix'] = calibration.get("35")
        CAL['VisOrigYmm'] = calibration.get("36")
        CAL['VisEndXpix'] = calibration.get("37")
        CAL['VisEndXmm'] = calibration.get("38")
        CAL['VisEndYpix'] = calibration.get("39")
        CAL['VisEndYmm'] = calibration.get("40")
        CAL['J1calOff'] = calibration.get("41")
        CAL['J2calOff'] = calibration.get("42")
        CAL['J3calOff'] = calibration.get("43")
        CAL['J4calOff'] = calibration.get("44")
        CAL['J5calOff'] = calibration.get("45")
        CAL['J6calOff'] = calibration.get("46")
        CAL['J1OpenLoopVal'] = calibration.get("47")
        CAL['J2OpenLoopVal'] = calibration.get("48")
        CAL['J3OpenLoopVal'] = calibration.get("49")
        CAL['J4OpenLoopVal'] = calibration.get("50")
        CAL['J5OpenLoopVal'] = calibration.get("51")
        CAL['J6OpenLoopVal'] = calibration.get("52")
        CAL['com2Port'] = calibration.get("53")
        CAL['curTheme'] = calibration.get("54")
        CAL['J1CalStatVal'] = calibration.get("55")
        CAL['J2CalStatVal'] = calibration.get("56")
        CAL['J3CalStatVal'] = calibration.get("57")
        CAL['J4CalStatVal'] = calibration.get("58")
        CAL['J5CalStatVal'] = calibration.get("59")
        CAL['J6CalStatVal'] = calibration.get("60")
        CAL['J7PosLim'] = calibration.get("61")
        CAL['J7rotation'] = calibration.get("62")
        CAL['J7steps'] = calibration.get("63")
        CAL['J7StepCur'] = calibration.get("64")
        CAL['J1CalStatVal2'] = calibration.get("65")
        CAL['J2CalStatVal2'] = calibration.get("66")
        CAL['J3CalStatVal2'] = calibration.get("67")
        CAL['J4CalStatVal2'] = calibration.get("68")
        CAL['J5CalStatVal2'] = calibration.get("69")
        CAL['J6CalStatVal2'] = calibration.get("70")
        CAL['VisBrightVal'] = calibration.get("71")
        CAL['VisContVal'] = calibration.get("72")
        CAL['VisBacColor'] = calibration.get("73")
        CAL['VisScore'] = calibration.get("74")
        CAL['VisX1Val'] = calibration.get("75")
        CAL['VisY1Val'] = calibration.get("76")
        CAL['VisX2Val'] = calibration.get("77")
        CAL['VisY2Val'] = calibration.get("78")
        CAL['VisRobX1Val'] = calibration.get("79")
        CAL['VisRobY1Val'] = calibration.get("80")
        CAL['VisRobX2Val'] = calibration.get("81")
        CAL['VisRobY2Val'] = calibration.get("82")
        CAL['zoom'] = calibration.get("83")
        CAL['pick180Val'] = calibration.get("84")
        CAL['pickClosestVal'] = calibration.get("85")
        CAL['curCam'] = calibration.get("86")
        CAL['fullRotVal'] = calibration.get("87")
        CAL['autoBGVal'] = calibration.get("88")
        CAL['mX1val'] = calibration.get("89")
        CAL['mY1val'] = calibration.get("90")
        CAL['mX2val'] = calibration.get("91")
        CAL['mY2val'] = calibration.get("92")
        CAL['J8length'] = calibration.get("93")
        CAL['J8rotation'] = calibration.get("94")
        CAL['J8steps'] = calibration.get("95")
        CAL['J9length'] = calibration.get("96")
        CAL['J9rotation'] = calibration.get("97")
        CAL['J9steps'] = calibration.get("98")
        CAL['J7calOff'] = calibration.get("99")
        CAL['J8calOff'] = calibration.get("100")
        CAL['J9calOff'] = calibration.get("101")
        CAL['GC_ST_E1'] = calibration.get("102")
        CAL['GC_ST_E2'] = calibration.get("103")
        CAL['GC_ST_E3'] = calibration.get("104")
        CAL['GC_ST_E4'] = calibration.get("105")
        CAL['GC_ST_E5'] = calibration.get("106")
        CAL['GC_ST_E6'] = calibration.get("107")
        CAL['GC_SToff_E1'] = calibration.get("108")
        CAL['GC_SToff_E2'] = calibration.get("109")
        CAL['GC_SToff_E3'] = calibration.get("110")
        CAL['GC_SToff_E4'] = calibration.get("111")
        CAL['GC_SToff_E5'] = calibration.get("112")
        CAL['GC_SToff_E6'] = calibration.get("113")
        CAL['DisableWristRotVal'] = calibration.get("114")
        CAL['J1MotDir'] = calibration.get("115")
        CAL['J2MotDir'] = calibration.get("116")
        CAL['J3MotDir'] = calibration.get("117")
        CAL['J4MotDir'] = calibration.get("118")
        CAL['J5MotDir'] = calibration.get("119")
        CAL['J6MotDir'] = calibration.get("120")
        CAL['J7MotDir'] = calibration.get("121")
        CAL['J8MotDir'] = calibration.get("122")
        CAL['J9MotDir'] = calibration.get("123")
        CAL['J1CalDir'] = calibration.get("124")
        CAL['J2CalDir'] = calibration.get("125")
        CAL['J3CalDir'] = calibration.get("126")
        CAL['J4CalDir'] = calibration.get("127")
        CAL['J5CalDir'] = calibration.get("128")
        CAL['J6CalDir'] = calibration.get("129")
        CAL['J7CalDir'] = calibration.get("130")
        CAL['J8CalDir'] = calibration.get("131")
        CAL['J9CalDir'] = calibration.get("132")
        CAL['J1PosLim'] = calibration.get("133")
        CAL['J1NegLim'] = calibration.get("134")
        CAL['J2PosLim'] = calibration.get("135")
        CAL['J2NegLim'] = calibration.get("136")
        CAL['J3PosLim'] = calibration.get("137")
        CAL['J3NegLim'] = calibration.get("138")
        CAL['J4PosLim'] = calibration.get("139")
        CAL['J4NegLim'] = calibration.get("140")
        CAL['J5PosLim'] = calibration.get("141")
        CAL['J5NegLim'] = calibration.get("142")
        CAL['J6PosLim'] = calibration.get("143")
        CAL['J6NegLim'] = calibration.get("144")
        CAL['J1StepDeg'] = calibration.get("145")
        CAL['J2StepDeg'] = calibration.get("146")
        CAL['J3StepDeg'] = calibration.get("147")
        CAL['J4StepDeg'] = calibration.get("148")
        CAL['J5StepDeg'] = calibration.get("149")
        CAL['J6StepDeg'] = calibration.get("150")
        CAL['J1DriveMS'] = calibration.get("151")
        CAL['J2DriveMS'] = calibration.get("152")
        CAL['J3DriveMS'] = calibration.get("153")
        CAL['J4DriveMS'] = calibration.get("154")
        CAL['J5DriveMS'] = calibration.get("155")
        CAL['J6DriveMS'] = calibration.get("156")
        CAL['J1EncCPR'] = calibration.get("157")
        CAL['J2EncCPR'] = calibration.get("158")
        CAL['J3EncCPR'] = calibration.get("159")
        CAL['J4EncCPR'] = calibration.get("160")
        CAL['J5EncCPR'] = calibration.get("161")
        CAL['J6EncCPR'] = calibration.get("162")
        CAL['J1ΘDHpar'] = calibration.get("163")
        CAL['J2ΘDHpar'] = calibration.get("164")
        CAL['J3ΘDHpar'] = calibration.get("165")
        CAL['J4ΘDHpar'] = calibration.get("166")
        CAL['J5ΘDHpar'] = calibration.get("167")
        CAL['J6ΘDHpar'] = calibration.get("168")
        CAL['J1αDHpar'] = calibration.get("169")
        CAL['J2αDHpar'] = calibration.get("170")
        CAL['J3αDHpar'] = calibration.get("171")
        CAL['J4αDHpar'] = calibration.get("172")
        CAL['J5αDHpar'] = calibration.get("173")
        CAL['J6αDHpar'] = calibration.get("174")
        CAL['J1dDHpar'] = calibration.get("175")
        CAL['J2dDHpar'] = calibration.get("176")
        CAL['J3dDHpar'] = calibration.get("177")
        CAL['J4dDHpar'] = calibration.get("178")
        CAL['J5dDHpar'] = calibration.get("179")
        CAL['J6dDHpar'] = calibration.get("180")
        CAL['J1aDHpar'] = calibration.get("181")
        CAL['J2aDHpar'] = calibration.get("182")
        CAL['J3aDHpar'] = calibration.get("183")
        CAL['J4aDHpar'] = calibration.get("184")
        CAL['J5aDHpar'] = calibration.get("185")
        CAL['J6aDHpar'] = calibration.get("186")
        CAL['GC_ST_WC'] = calibration.get("187")
        CAL['J7CalStatVal'] = calibration.get("188")
        CAL['J8CalStatVal'] = calibration.get("189")
        CAL['J9CalStatVal'] = calibration.get("190")
        CAL['J7CalStatVal2'] = calibration.get("191")
        CAL['J8CalStatVal2'] = calibration.get("192")
        CAL['J9CalStatVal2'] = calibration.get("193")
        CAL['setColor'] = calibration.get("194")
    
    except Exception as e:
        logger.error(f"Error converting calibration: {e}")
        return False
    
    try:
        save_calibration(CAL)
    except Exception as e:
        logger.error(f"Error saving new calibration: {e}")
        return False
    try:
        logger.info("Backing up ARbot.cal to Arbot.cal.bak")
        os.rename('ARbot.cal', 'Arbot.cal.bak')
    except Exception as e:
        logger.error(f"Error renaming file: {e}")
        return False
    
    return CAL
