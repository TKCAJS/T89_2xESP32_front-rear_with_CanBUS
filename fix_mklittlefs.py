Import("env")
import os, glob

packages_dir = os.path.join(os.path.expanduser("~"), ".platformio", "packages")

# Find mklittlefs.exe in any tool-mklittlefs package directory
candidates = glob.glob(os.path.join(packages_dir, "tool-mklittlefs*", "mklittlefs.exe"))
if candidates:
    tool_exe = candidates[0]
    tool_dir = os.path.dirname(tool_exe)
    env.Replace(MKFSTOOL=tool_exe)
    env.PrependENVPath("PATH", tool_dir)
    os.environ["PATH"] = tool_dir + os.pathsep + os.environ.get("PATH", "")
    print("mklittlefs: using", tool_exe)
else:
    print("WARNING: mklittlefs.exe not found in", packages_dir)
