package com.team973.frc2025;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.json.JsonReadFeature;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.team973.frc2025.shared.RobotInfo;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class RobotConfig {

  private static boolean m_initialized;
  private static RobotInfo m_robotInfo;

  // The following are set once during initialization so that we can log them
  private static String m_configProfile;

  public static RobotInfo get() {
    maybeInitialize();
    return m_robotInfo;
  }

  public static String getConfigProfile() {
    maybeInitialize();
    return m_configProfile;
  }

  private static synchronized void maybeInitialize() {
    if (m_initialized) {
      return;
    }
    List<String> configPaths = getConfigPaths();
    System.err.printf("Using configSources: %s\n", configPaths);
    m_robotInfo = getMergedConfig(configPaths);
    m_initialized = true;
  }

  public static String readConfigProfile() throws IOException {
    String path = Filesystem.getOperatingDirectory().toPath().resolve("profile").toString();
    return Files.readString(Paths.get(path));
  }

  public static RobotInfo getMergedConfig(List<String> configPaths) {
    RobotInfo res = new RobotInfo();

    for (String path : configPaths) {

      ObjectMapper mapper = new ObjectMapper();
      mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, true);
      mapper.configure(JsonReadFeature.ALLOW_TRAILING_COMMA.mappedFeature(), true);
      mapper.setDefaultMergeable(true);
      mapper.configure(JsonParser.Feature.ALLOW_COMMENTS, true);
      mapper.configOverride(List.class).setMergeable(false);
      try {
        mapper.readerForUpdating(res).readValue(new File(path));
      } catch (com.fasterxml.jackson.core.JsonParseException e) {
        System.err.printf("JSON parsing error in %s: %s\n", path, e.getMessage());
        throw new RuntimeException(e);
      } catch (IOException e) {
        System.err.println("Could not read file: " + e.getMessage());
        throw new RuntimeException(e);
      }
    }

    return res;
  }

  /** configSuffix is something like "common", "simulation", */
  public static String absPathFromConfigSuffix(String configSuffix) {
    return Filesystem.getDeployDirectory()
        .toPath()
        .resolve("config-" + configSuffix.trim() + ".jsonc")
        .toString();
  }

  public static List<String> getConfigPaths() {
    List<String> res = new ArrayList<>();

    // Every robot starts out with the base config (config-common.yaml)
    res.add(absPathFromConfigSuffix("common"));

    if (RobotBase.isSimulation()) {
      m_configProfile = "simulation";
      res.add(absPathFromConfigSuffix("simulation"));
    } else {
      try {
        m_configProfile = readConfigProfile();
        res.add(absPathFromConfigSuffix(m_configProfile));
      } catch (IOException e) {
        // On a competition robot this isn't really a big deal because the common
        // config should be complete, so we can continue on here. Setting the
        // config profile to ERROR will signal that something failed during setup.
        System.err.printf("Failed to read config profile: %s\n", e.getMessage());
        m_configProfile = "ERROR";
      }
    }

    return res;
  }
}
