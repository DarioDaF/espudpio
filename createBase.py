
import os, shutil

TOUCH_FILES = [ 'include/defSettings.hpp' ]

for f in TOUCH_FILES:
    if not os.path.isfile(f):
        shutil.copy(f + '.base', f)
