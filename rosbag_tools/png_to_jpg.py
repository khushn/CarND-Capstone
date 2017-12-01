#!/usr/bin/env python

import glob
import re
from PIL import Image

f = glob.glob("../training_images/data/*/*.png")

for fname in f:
	fname2 = re.sub('.png', '.jpg', fname)
	print 'Converting ', fname, 'to', fname2
	im = Image.open(fname)
	rgb_im = im.convert('RGB')
	rgb_im.save(fname2)


print 'Num of files: ', len(f)
