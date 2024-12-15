#pragma once

#include <frc/Filesystem.h>

#include <map>

#include "frc846/math/fieldpoints.h"

enum AutoFlipType { kNone, kMirror, kMirrorOnlyY };

struct AutoData {
  std::string name;
  AutoFlipType red;
  AutoFlipType blue;
  frc846::math::FieldPoint start;
  std::vector<std::variant<std::vector<frc846::math::FieldPoint>, std::string>>
      actions;
};

class Field_nonstatic : public frc846::base::Loggable {
 public:
  std::vector<std::pair<std::string, frc846::math::FieldPoint>> points;

  std::vector<std::pair<std::string, std::vector<frc846::math::FieldPoint>>>
      paths;

  frc846::math::FieldPoint getPoint(std::string name);

  std::vector<frc846::math::FieldPoint> getPath(std::string name);

  Field_nonstatic();

  void Setup();

  std::vector<AutoData> getAllAutoData();

 private:
  void addPoint(std::string name, frc846::math::FieldPoint point) {
    points.push_back(std::pair{name, point});
  }

  static std::vector<std::string> split(const std::string& s, char delimiter);

  static std::vector<std::string> readLines(std::string filename);

  static std::string fixPath(std::string path);

  static std::string forceNormalPath(std::string path);

  static std::string getFileDirectory();

  frc846::math::FieldPoint parsePoint(std::string line);

  void readPointsFile();

  void readAllPaths();
};

class Field {
 public:
  static frc846::math::FieldPoint getPoint(std::string name) {
    return instance.getPoint(name);
  }

  static std::vector<frc846::math::FieldPoint> getPath(std::string name) {
    return instance.getPath(name);
  }

  static void Setup() { instance.Setup(); }

  static std::vector<AutoData> getAllAutoData() {
    return instance.getAllAutoData();
  }

 private:
  static Field_nonstatic instance;
};