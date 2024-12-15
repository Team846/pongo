#include "field.h"

#include <frc/Filesystem.h>

#include <exception>

frc846::math::FieldPoint Field_nonstatic::getPoint(std::string name) {
  if (!points.empty()) {
    for (auto& point : points) {
      if (point.first == name) {
        return point.second;
      }
    }
  }
  Warn("Unable to access fieldpoint: {}.", name);
  return frc846::math::FieldPoint{{0_in, 0_in}, 0_deg, {0_fps, 0_fps}};
}

std::vector<frc846::math::FieldPoint> Field_nonstatic::getPath(
    std::string name) {
  for (auto& path : paths) {
    if (path.first == name) {
      return path.second;
    }
  }
  Warn("Unable to access path: {}.", name);
  return {};
}

std::vector<std::string> Field_nonstatic::split(const std::string& s,
                                                char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

std::vector<std::string> Field_nonstatic::readLines(std::string filename) {
  std::ifstream file;
  file.open(filename);

  std::vector<std::string> lines{};

  std::string content_string;
  while (file.good()) {
    file >> content_string;
    lines.push_back(content_string);
  }

  return lines;
}

std::string Field_nonstatic::fixPath(std::string path) {
#ifdef _WIN32
  for (char& ch : path) {
    if (ch == '/') {
      ch = '\\';
    }
  }
  return path;
#else
  for (char& ch : path) {
    if (ch == '\\') {
      ch = '/';
    }
  }
  return path;
#endif
}

std::string Field_nonstatic::forceNormalPath(std::string path) {
  for (char& ch : path) {
    if (ch == '\\') {
      ch = '/';
    }
  }
  return path;
}

std::string Field_nonstatic::getFileDirectory() {
#ifdef _WIN32
  std::string deploy_dir = frc::filesystem::GetDeployDirectory();

  std::vector<std::string> tokens = split(deploy_dir, '\\');
  std::string rejoined{};
  for (size_t i = 0; i < tokens.size(); i++) {
    if (tokens[i] != "main") {
      rejoined += tokens[i];
      rejoined += "/";
    }
  }

  return fixPath(rejoined + "autos/");

#else

  return "/home/lvuser/deploy/autos/";
#endif
}

frc846::math::FieldPoint Field_nonstatic::parsePoint(std::string line) {
  frc846::math::FieldPoint pt{{0_in, 0_in}, 0_deg, {0_fps, 0_fps}};

  auto tokens = split(line, ',');
  if (tokens.size() != 0) {
    if (tokens[0] == "N") {
      auto v = 0_fps;
      if (tokens.size() == 6) v = std::stod(tokens[5]) * 1_fps;

      pt = {{std::stod(tokens[2]) * 1_in, std::stod(tokens[3]) * 1_in},
            std::stod(tokens[4]) * 1_deg,
            {v, 0_fps}};
    } else if (tokens[0] == "P") {
      auto v = 0_fps;
      if (tokens.size() == 5) v = std::stod(tokens[4]) * 1_fps;

      pt = {{std::stod(tokens[1]) * 1_in, std::stod(tokens[2]) * 1_in},
            std::stod(tokens[3]) * 1_deg,
            {v, 0_fps}};
    } else if (tokens[0] == "F") {
      if (tokens.size() == 2) {
        pt = getPoint(tokens[1]);
      }
    } else {
      Warn("Unable to parse point from raw: {}.", line);
    }
  }
  return pt;
}

void Field_nonstatic::readPointsFile() {
  auto pts_us = readLines(Field_nonstatic::getFileDirectory() + "points.lst");

  for (auto& pt : pts_us) {
    auto tokens = split(pt, ',');
    if (tokens.size() >= 2) {
      addPoint(tokens[1], parsePoint(pt));
      Log("Added point: {}.", tokens[1]);
    } else {
      Warn("Invalid point: {}.", pt);
    }
  }
}

void Field_nonstatic::readAllPaths() {
  std::vector<std::string> path_files;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(
             Field_nonstatic::getFileDirectory() + "paths")) {
      if (entry.is_regular_file()) path_files.push_back(entry.path().string());
      Log("Found path file: {}.", entry.path().string());
    }
  } catch (const std::exception& exc) {
    (void)exc;
  }

  for (const auto& filename : path_files) {
    auto path_us = readLines(filename);
    std::vector<frc846::math::FieldPoint> path;
    for (auto& pt : path_us) {
      path.push_back(parsePoint(pt));
    }
    auto split_filename = split(forceNormalPath(filename), '/');

    paths.push_back(
        std::pair{split_filename.at(split_filename.size() - 1), path});
    Log("Added path: {}.", split_filename.at(split_filename.size() - 1));
  }
}

Field_nonstatic::Field_nonstatic() : Loggable{"Field"}, points{}, paths{} {}

void Field_nonstatic::Setup() {
  readPointsFile();
  readAllPaths();
};

std::vector<AutoData> Field_nonstatic::getAllAutoData() {
  std::vector<AutoData> result;

  std::vector<std::string> auto_files;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(
             Field_nonstatic::getFileDirectory() + "scripts")) {
      if (entry.is_regular_file()) auto_files.push_back(entry.path().string());
    }
  } catch (const std::exception& exc) {
    (void)exc;
  }

  for (const auto& filename : auto_files) {
    auto auto_lines = readLines(filename);

    if (auto_lines.size() < 2) {
      Warn("Invalid auto file: {}.", filename);
      continue;
    }

    auto flips = split(auto_lines[0], ',');

    AutoFlipType red = (AutoFlipType)std::stoi(flips[0]);
    AutoFlipType blue = (AutoFlipType)std::stoi(flips[1]);

    auto start_point = parsePoint(auto_lines[1]);

    auto_lines.erase(auto_lines.begin(), auto_lines.begin() + 1);

    std::vector<
        std::variant<std::vector<frc846::math::FieldPoint>, std::string>>
        actions;

    for (auto& line : auto_lines) {
      auto split_line = split(line, ',');
      if (split_line.size() != 2) {
        Warn("Invalid auto action: {}.", line);
        continue;
      }
      if (split_line[0] == "PATH") {
        actions.push_back(getPath(split_line[1]));
      } else if (split_line[0] == "ACT") {
        actions.push_back(split_line[1]);
      }
    }

    Log("Added auto: {}.", filename);

    Log("Auto has {} actions.", actions.size());

    auto split_filename = split(forceNormalPath(filename), '/');
    auto auto_name = split_filename.at(split_filename.size() - 1);

    result.push_back({auto_name, red, blue, start_point, actions});
  }

  return result;
}

Field_nonstatic Field::instance{};