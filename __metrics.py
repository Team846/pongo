# Outputs metrics such as [linecount] for this codebase

import os

INCL = ['*.py', '*.cc', '*.cpp', '.h', '.hpp', '.gradle', '.bat', '.sh', '.md', '.standard', '.json', '.lst']
FORCE_INCL_DIR = ["src\\deploy", "src/deploy"]

if __name__ == "__main__":

    cwd = os.getcwd()

    files = []
    for root, dirs, filenames in os.walk(cwd):
        for filename in filenames:
            if any([root.__contains__(d) for d in FORCE_INCL_DIR]):
                files.append(os.path.join(root, filename))
                continue
            for ext in INCL:
                if filename.endswith(ext):
                    files.append(os.path.join(root, filename))

    linecount = 0
    for file in files:
        linecount += sum(1 for line in open(file))

    print(linecount)