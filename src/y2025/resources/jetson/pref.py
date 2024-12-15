from networktables import NetworkTable
from networktables.util import NetworkTablesInstance
import subprocess

class NumericStore:
    def __init__(self):
        self.store = {}
        self.read()

    def read(self):
        print("Reading prefs.")
        try:
            save_file = open("prefs.numeric", "r")
            prefs = save_file.readlines()
            for x in prefs:
                key, value = x.split(":")
                self.store[key] = float(value)
            save_file.close()
        except:
            print("Error reading prefs (numeric).")

    def save(self):
        print("Save triggered.")
        save_file = open("prefs.numeric", "w")
        for key in self.store:
            save_file.write(f"{key}:{round(self.store[key], 2)}\n")
        save_file.close()

    def put(self, key: str, value: float):
        self.store[key] = value
        self.save()

    def get(self, key: str) -> float:
        if self.store[key] is None:
            return 0.0
        return self.store[key]
    
    def has(self, key: str) -> bool:
        return key in self.store
    
class BooleanStore:
    def __init__(self):
        self.store = {}
        self.read()

    def read(self):
        print("Reading prefs.")
        try:
            save_file = open("prefs.boolean", "r")
            prefs = save_file.readlines()
            for x in prefs:
                key, value = x.split(":")
                self.store[key] = bool(value)
            save_file.close()
        except:
            print("Error reading prefs (boolean).")

    def save(self):
        print("Save triggered.")
        save_file = open("prefs.boolean", "w")
        for key in self.store:
            save_file.write(f"{key}:{self.store[key]}\n")
        save_file.close()

    def put(self, key: str, value: bool):
        self.store[key] = value
        self.save()

    def get(self, key: str) -> bool:
        if self.store[key] is None:
            return False
        return self.store[key]
    
    def has(self, key: str) -> bool:
        return key in self.store

class NumericPref:
    store = NumericStore()

    @staticmethod
    def valueChanged(table, key, value, isNew):
        print(f"Numeric Value changed: {key} -> {value}.")
        NumericPref.store.put(key, float(value))

    def __init__(self, table: NetworkTable, key: str, defaultValue: float):
        self.defaultValue = defaultValue
        self.key = key

        self.entry = table.getEntry(key)

        if (NumericPref.store.has(key) is False):
            NumericPref.store.put(key, defaultValue)
        self.entry.forceSetDouble(NumericPref.store.get(key))

        self.entry.addListener(self.valueChanged, NetworkTablesInstance.NotifyFlags.UPDATE)

        self.entry.setPersistent()

    def get(self) -> float:
        return self.entry.getDouble(self.defaultValue)
    
class BooleanPref:
    store = BooleanStore()

    @staticmethod
    def valueChanged(table, key, value, isNew):
        print(f"Boolean Value changed: {key} -> {value}.")
        BooleanPref.store.put(key, bool(value))

    def __init__(self, table: NetworkTable, key: str, defaultValue: bool):
        self.defaultValue = defaultValue
        self.key = key

        self.entry = table.getEntry(key)

        if (BooleanPref.store.has(key) is False):
            BooleanPref.store.put(key, defaultValue)
        self.entry.forceSetBoolean(BooleanPref.store.get(key))

        self.entry.addListener(self.valueChanged, NetworkTablesInstance.NotifyFlags.UPDATE)

        self.entry.setPersistent()

    def get(self) -> bool:
        return self.entry.getBoolean(self.defaultValue)

class KillSwitch:
    @staticmethod
    def reboot(table, key, value, isNew):
        if bool(value):
            subprocess.run(["reboot"])

    def __init__(self, table: NetworkTable):
        self.entry = table.getEntry("kill")
        self.entry.forceSetBoolean(False)

        self.entry.addListener(self.reboot, NetworkTablesInstance.NotifyFlags.UPDATE)