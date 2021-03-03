#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate
import matplotlib.patches as mpatches
import glob
import os
from sklearn.metrics.pairwise import cosine_similarity


plt.rcParams['font.size'] = '22'
labels = ['','similar objects', 'different objets','']
x = np.arange(len(labels))  # the label locations

pathsimilar = '/home/rin/MT-plotting/PlotPy/../../Desktop/Plot-Master-T/Experiment-1-2/albi-orange/albi-albi/'
for filename in glob.glob(os.path.join(pathsimilar, '*.txt')):
    with open(os.path.join(os.getcwd(), filename), 'r') as f:  # open in readonly mode

        # -----------------------------------------------------------------------------------------Colorfit
        if f.name.endswith("ColorFit_all.txt"):
            lines = f.readlines()
            Similar_ColorFit_all = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFH

        elif f.name.endswith("PFH_max_corr.txt"):
            lines = f.readlines()
            Similar_PFH_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_all.txt"):
            lines = f.readlines()
            Similar_PFH_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_PFH_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_all.txt"):
            lines = f.readlines()
            Similar_PFH_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_corr.txt"):
            lines = f.readlines()
            Similar_PFH_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_upper.txt"):
            lines = f.readlines()
            Similar_PFH_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFHRGB

        elif f.name.endswith("PFHRGB_max_corr.txt"):
            lines = f.readlines()
            Similar_PFHRGB_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_all.txt"):
            lines = f.readlines()
            Similar_PFHRGB_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_PFHRGB_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_all.txt"):
            lines = f.readlines()
            Similar_PFHRGB_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_corr.txt"):
            lines = f.readlines()
            Similar_PFHRGB_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_upper.txt"):
            lines = f.readlines()
            Similar_PFHRGB_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_sqrt.txt"):
            lines = f.readlines()
            Similar_PFHRGB_sqrt = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("PFHRGB_anzahlDp1.txt"):
            lines = f.readlines()
            Similar_PFHRGB_dp1 = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("PFHRGB_anzahlDp2.txt"):
            lines = f.readlines()
            Similar_PFHRGB_dp2 = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxALL.txt"):
            lines = f.readlines()
            Similar_PFHRGB_mALL = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxCor.txt"):
            lines = f.readlines()
            Similar_PFHRGB_mC = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxCorRE.txt"):
            lines = f.readlines()
            Similar_PFHRGB_mR = [float(line.split()[0]) for line in lines]


        # -----------------------------------------------------------------------------------------shape

        elif f.name.endswith("ShapeContext_max_corr.txt"):
            lines = f.readlines()
            Similar_ShapeContext_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_all.txt"):
            lines = f.readlines()
            Similar_ShapeContext_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_ShapeContext_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_all.txt"):
            lines = f.readlines()
            Similar_ShapeContext_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_corr.txt"):
            lines = f.readlines()
            Similar_ShapeContext_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_upper.txt"):
            lines = f.readlines()
            Similar_ShapeContext_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shape

        elif f.name.endswith("ShotColor_maxWD.txt"):
            lines = f.readlines()
            Similar_ShotColor_maxWD = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_corr.txt"):
            lines = f.readlines()
            Similar_ShotColor_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_all.txt"):
            lines = f.readlines()
            Similar_ShotColor_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_ShotColor_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_all.txt"):
            lines = f.readlines()
            Similar_ShotColor_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_corr.txt"):
            lines = f.readlines()
            Similar_ShotColor_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_upper.txt"):
            lines = f.readlines()
            Similar_ShotColor_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_sqrt.txt"):
            lines = f.readlines()
            Similar_ShotColor_sqrt = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("ShotColor_anzahlDp1.txt"):
            lines = f.readlines()
            Similar_ShotColor_dp1 = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("ShotColor_anzahlDp2.txt"):
            lines = f.readlines()
            Similar_ShotColor_dp2 = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxALL.txt"):
            lines = f.readlines()
            Similar_ShotColor_mALL = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxCor.txt"):
            lines = f.readlines()
            Similar_ShotColor_mC = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxCorRE.txt"):
            lines = f.readlines()
            Similar_ShotColor_mR = [float(line.split()[0]) for line in lines]


        # -----------------------------------------------------------------------------------------shot

        elif f.name.endswith("USC_max_corr.txt"):
            lines = f.readlines()
            Similar_USC_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_all.txt"):
            lines = f.readlines()
            Similar_USC_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_USC_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_all.txt"):
            lines = f.readlines()
            Similar_USC_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_corr.txt"):
            lines = f.readlines()
            Similar_USC_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_upper.txt"):
            lines = f.readlines()
            Similar_USC_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------rsd
        elif f.name.endswith("rsd_max_corr.txt"):
            lines = f.readlines()
            Similar_rsd_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_all.txt"):
            lines = f.readlines()
            Similar_rsd_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_rsd_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_all.txt"):
            lines = f.readlines()
            Similar_rsd_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_corr.txt"):
            lines = f.readlines()
            Similar_rsd_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_upper.txt"):
            lines = f.readlines()
            Similar_rsd_average_upper = [float(line.split()[0]) for line in lines]

        # -----------------------------------------------------------------------------------------rsd

        elif f.name.endswith("rsd_max_corr.txt"):
            lines = f.readlines()
            Similar_rsd_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_all.txt"):
            lines = f.readlines()
            Similar_rsd_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_remain_corr.txt"):
            lines = f.readlines()
            Similar_rsd_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_all.txt"):
            lines = f.readlines()
            Similar_rsd_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_corr.txt"):
            lines = f.readlines()
            Similar_rsd_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_upper.txt"):
            lines = f.readlines()
            Similar_rsd_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_sqrt.txt"):
            lines = f.readlines()
            Similar_rsd_sqrt = [float(line.split()[0]) for line in lines]

            # -----------------------------------------------------------------------------------------rsd
        elif f.name.endswith("RSD_anzahlDp1.txt"):
            lines = f.readlines()
            Similar_RSD_dp1 = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataNN.txt"):
            lines = f.readlines()
            Similar_DataNN = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataIndex.txt"):
            lines = f.readlines()
            Similar_DataIndex = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataNNPlane.txt"):
            lines = f.readlines()
            Similar_DataNNPlane = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataRift.txt"):
            lines = f.readlines()
            Similar_DataRift = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataHausdorf.txt"):
            lines = f.readlines()
            Similar_DataHausdorf = [float(line.split()[0]) for line in lines]

pathdifferent = '/home/rin/MT-plotting/PlotPy/../../Desktop/Plot-Master-T/Experiment-1-2/albi-orange/albi-orange/'
for filename in glob.glob(os.path.join(pathdifferent, '*.txt')):
    with open(os.path.join(os.getcwd(), filename), 'r') as f:  # open in readonly mode
        # -----------------------------------------------------------------------------------------Colorfit
        if f.name.endswith("ColorFit_all.txt"):
            lines = f.readlines()
            Different_ColorFit_all = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFH

        elif f.name.endswith("PFH_max_corr.txt"):
            lines = f.readlines()
            Different_PFH_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_all.txt"):
            lines = f.readlines()
            Different_PFH_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_remain_corr.txt"):
            lines = f.readlines()
            Different_PFH_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_all.txt"):
            lines = f.readlines()
            Different_PFH_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_corr.txt"):
            lines = f.readlines()
            Different_PFH_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_upper.txt"):
            lines = f.readlines()
            Different_PFH_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFHRGB

        elif f.name.endswith("PFHRGB_max_corr.txt"):
            lines = f.readlines()
            Different_PFHRGB_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_all.txt"):
            lines = f.readlines()
            Different_PFHRGB_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_remain_corr.txt"):
            lines = f.readlines()
            Different_PFHRGB_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_all.txt"):
            lines = f.readlines()
            Different_PFHRGB_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_corr.txt"):
            lines = f.readlines()
            Different_PFHRGB_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_upper.txt"):
            lines = f.readlines()
            Different_PFHRGB_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_sqrt.txt"):
            lines = f.readlines()
            Different_PFHRGB_sqrt = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_anzahlDp1.txt"):
            lines = f.readlines()
            Diff_PFHRGB_dp1 = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("PFHRGB_anzahlDp2.txt"):
            lines = f.readlines()
            Diff_PFHRGB_dp2 = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxALL.txt"):
            lines = f.readlines()
            Diff_PFHRGB_mALL = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxCor.txt"):
            lines = f.readlines()
            Diff_PFHRGB_mC = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_maxCorRE.txt"):
            lines = f.readlines()
            Diff_PFHRGB_mR = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shape

        elif f.name.endswith("ShapeContext_max_corr.txt"):
            lines = f.readlines()
            Different_ShapeContext_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_all.txt"):
            lines = f.readlines()
            Different_ShapeContext_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_remain_corr.txt"):
            lines = f.readlines()
            Different_ShapeContext_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_all.txt"):
            lines = f.readlines()
            Different_ShapeContext_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_corr.txt"):
            lines = f.readlines()
            Different_ShapeContext_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_upper.txt"):
            lines = f.readlines()
            Different_ShapeContext_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shape
        elif f.name.endswith("ShotColor_maxWD.txt"):
            lines = f.readlines()
            Different_ShotColor_maxWD = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_corr.txt"):
            lines = f.readlines()
            Different_ShotColor_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_all.txt"):
            lines = f.readlines()
            Different_ShotColor_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_remain_corr.txt"):
            lines = f.readlines()
            Different_ShotColor_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_all.txt"):
            lines = f.readlines()
            Different_ShotColor_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_corr.txt"):
            lines = f.readlines()
            Different_ShotColor_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_upper.txt"):
            lines = f.readlines()
            Different_ShotColor_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_sqrt.txt"):
            lines = f.readlines()
            Different_ShotColor_sqrt = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("ShotColor_anzahlDp1.txt"):
            lines = f.readlines()
            Diff_ShotColor_dp1 = [float(line.split()[0]) for line in lines]

        elif f.name.endswith("ShotColor_anzahlDp2.txt"):
            lines = f.readlines()
            Diff_ShotColor_dp2 = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxALL.txt"):
            lines = f.readlines()
            Diff_ShotColor_mALL = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxCor.txt"):
            lines = f.readlines()
            Diff_ShotColor_mC = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_maxCorRE.txt"):
            lines = f.readlines()
            Diff_ShotColor_mR = [float(line.split()[0]) for line in lines]

        # -----------------------------------------------------------------------------------------shot

        elif f.name.endswith("USC_max_corr.txt"):
            lines = f.readlines()
            Different_USC_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_all.txt"):
            lines = f.readlines()
            Different_USC_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_remain_corr.txt"):
            lines = f.readlines()
            Different_USC_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_all.txt"):
            lines = f.readlines()
            Different_USC_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_corr.txt"):
            lines = f.readlines()
            Different_USC_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_upper.txt"):
            lines = f.readlines()
            Different_USC_average_upper = [float(line.split()[0]) for line in lines]

            # -----------------------------------------------------------------------------------------shot
        # -----------------------------------------------------------------------------------------rsd

        elif f.name.endswith("rsd_max_corr.txt"):
            lines = f.readlines()
            Different_rsd_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_all.txt"):
            lines = f.readlines()
            Different_rsd_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_remain_corr.txt"):
            lines = f.readlines()
            Different_rsd_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_all.txt"):
            lines = f.readlines()
            Different_rsd_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_corr.txt"):
            lines = f.readlines()
            Different_rsd_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_upper.txt"):
            lines = f.readlines()
            Different_rsd_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_sqrt.txt"):
            lines = f.readlines()
            Different_rsd_sqrt = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("RSD_anzahlDp1.txt"):
            lines = f.readlines()
            Diff_RSD_dp1 = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------rsd
        elif f.name.endswith("DataNN.txt"):
            lines = f.readlines()
            Different_DataNN = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataIndex.txt"):
            lines = f.readlines()
            Different_DataIndex = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataNNPlane.txt"):
            lines = f.readlines()
            Different_DataNNPlane = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataRift.txt"):
            lines = f.readlines()
            Different_DataRift = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("DataHausdorf.txt"):
            lines = f.readlines()
            Different_DataHausdorf = [float(line.split()[0]) for line in lines]

pathTexture = '/home/rin/MT-plotting/PlotPy/../../cat_ws/'
for filename in glob.glob(os.path.join(pathTexture, '*.txt')):
    with open(os.path.join(os.getcwd(), filename), 'r') as f:  # open in readonly mode
        # -----------------------------------------------------------------------------------------Colorfit
        if f.name.endswith("ColorFit_all.txt"):
            lines = f.readlines()
            Texture_ColorFit_all = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFH

        elif f.name.endswith("PFH_max_corr.txt"):
            lines = f.readlines()
            Texture_PFH_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_all.txt"):
            lines = f.readlines()
            Texture_PFH_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_PFH_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_all.txt"):
            lines = f.readlines()
            Texture_PFH_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_corr.txt"):
            lines = f.readlines()
            Texture_PFH_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFH_average_upper.txt"):
            lines = f.readlines()
            Texture_PFH_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------PFHRGB

        elif f.name.endswith("PFHRGB_max_corr.txt"):
            lines = f.readlines()
            Texture_PFHRGB_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_all.txt"):
            lines = f.readlines()
            Texture_PFHRGB_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_PFHRGB_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_all.txt"):
            lines = f.readlines()
            Texture_PFHRGB_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_corr.txt"):
            lines = f.readlines()
            Texture_PFHRGB_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_average_upper.txt"):
            lines = f.readlines()
            Texture_PFHRGB_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("PFHRGB_sqrt.txt"):
            lines = f.readlines()
            Texture_PFHRGB_sqrt = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shape

        elif f.name.endswith("ShapeContext_max_corr.txt"):
            lines = f.readlines()
            Texture_ShapeContext_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_all.txt"):
            lines = f.readlines()
            Texture_ShapeContext_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_ShapeContext_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_all.txt"):
            lines = f.readlines()
            Texture_ShapeContext_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_corr.txt"):
            lines = f.readlines()
            Texture_ShapeContext_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShapeContext_average_upper.txt"):
            lines = f.readlines()
            Texture_ShapeContext_average_upper = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shape

        elif f.name.endswith("ShotColor_max_corr.txt"):
            lines = f.readlines()
            Texture_ShotColor_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_all.txt"):
            lines = f.readlines()
            Texture_ShotColor_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_ShotColor_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_all.txt"):
            lines = f.readlines()
            Texture_ShotColor_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_corr.txt"):
            lines = f.readlines()
            Texture_ShotColor_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_average_upper.txt"):
            lines = f.readlines()
            Texture_ShotColor_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("ShotColor_sqrt.txt"):
            lines = f.readlines()
            Texture_ShotColor_sqrt = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("albiturned.txt"):
            lines = f.readlines()
            Texture_ShotColor_dp1 = [float(line.split()[0]) for line in lines]
        # -----------------------------------------------------------------------------------------shot

        elif f.name.endswith("USC_max_corr.txt"):
            lines = f.readlines()
            Texture_USC_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_all.txt"):
            lines = f.readlines()
            Texture_USC_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_USC_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_all.txt"):
            lines = f.readlines()
            Texture_USC_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_corr.txt"):
            lines = f.readlines()
            Texture_USC_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("USC_average_upper.txt"):
            lines = f.readlines()
            Texture_USC_average_upper = [float(line.split()[0]) for line in lines]

            # -----------------------------------------------------------------------------------------shot
        # -----------------------------------------------------------------------------------------rsd

        elif f.name.endswith("rsd_max_corr.txt"):
            lines = f.readlines()
            Texture_rsd_max_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_all.txt"):
            lines = f.readlines()
            Texture_rsd_max_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_max_remain_corr.txt"):
            lines = f.readlines()
            Texture_rsd_max_remain_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_all.txt"):
            lines = f.readlines()
            Texture_rsd_average_all = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_corr.txt"):
            lines = f.readlines()
            Texture_rsd_average_corr = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_average_upper.txt"):
            lines = f.readlines()
            Texture_rsd_average_upper = [float(line.split()[0]) for line in lines]
        elif f.name.endswith("rsd_sqrt.txt"):
            lines = f.readlines()
            Texture_rsd_sqrt = [float(line.split()[0]) for line in lines]


def normalize_my_data(similarData, differentData, bool, textureData=[1]):
    dataCompact = []
    dataCompact.extend(similarData)
    dataCompact.extend(differentData)
    
    minA = min(dataCompact)
    maxA = max(dataCompact)

    if bool == True:
        similar_normed = []
        for i in similarData:
            similar_normed.append(((i - minA) / (maxA - minA)) * 1)

        different_normed = []
        for i in differentData:
            different_normed.append(((i - minA) / (maxA - minA)) * 1)
    elif bool == False:
        similar_normed = []
        for i in similarData:
            similar_normed.append(((i - maxA) / (minA - maxA)) * 1)

        different_normed = []
        for i in differentData:
            different_normed.append(((i - maxA) / (minA - maxA)) * 1)



    tNormedData = []
    for i in textureData:
        tNormedData.append(((i - minA) / (maxA - minA)) * 1)

    return similar_normed, different_normed, tNormedData


def linesForOneMethodOnly(datanormed, colorthis, labelthis):
    y = datanormed
    x = np.linspace(0, u, len(y))
    plt.plot(x, y, color=colorthis, label=labelthis)


u = 60


def shotDP1():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_dp1, Diff_ShotColor_dp1,
                                                                      True, Texture_ShotColor_dp1)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar object')
    linesForOneMethodOnly(different_normed, 'red', 'different object')
    linesForOneMethodOnly(tNormedData, 'green', 'partial similar object')

    np.savetxt('similar_shot.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_shot.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_shot_normed
    similar_shot_normed = similar_normed
    global different_shot_normed
    different_shot_normed = different_normed


    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.axhline(y=0.46830501, color='grey', linestyle='-')
    plt.text(64,0.46830501,'0.46830501 \n threshold', fontsize=18)
    plt.xlabel('iteration')

    plt.ylabel('correspondence')

    plt.title('Signatures of Histograms of Orientations and Color (SHOTColor)')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()

def pfhrgbDP1():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_dp1, Diff_PFHRGB_dp1,
                                                                      True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar object')
    linesForOneMethodOnly(different_normed, 'red', 'different object')


    np.savetxt('similar_pfhrgb.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_pfhrgb.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_pfhrgb_normed
    similar_pfhrgb_normed = similar_normed
    global different_pfhrgb_normed
    different_pfhrgb_normed = different_normed
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))

    plt.xlabel('iteration')

    plt.ylabel('correspondence')
    plt.axhline(y=0.44550737, color='grey', linestyle='-')

    plt.text(540,0.44550737,'0.44550737 \n threshold', fontsize=18)
    plt.title('Point Feature Histogram-RGB (PFHRGB)')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()

def RSDDP1():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_RSD_dp1, Diff_RSD_dp1,
                                                                      True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar object')
    linesForOneMethodOnly(different_normed, 'red', 'different object')

    np.savetxt('similar_rsd.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_rsd.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_rsd_normed
    similar_rsd_normed = similar_normed
    global different_rsd_normed
    different_rsd_normed = different_normed
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.axhline(y=0.52859919, color='grey', linestyle='-')

    plt.text(540,0.52859919,'0.52859919 \n threshold', fontsize=18)
    plt.xlabel('iteration')

    plt.ylabel('correspondence')

    plt.title('Radius-Based Surface Descriptor (RSD) ')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


shotDP1()
#pfhrgbDP1()
#RSDDP1()

def DataHausdorf():
    similar_normed, different_normed, t = normalize_my_data(Similar_DataHausdorf,
                                                            Different_DataHausdorf, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar objets-true case')
    linesForOneMethodOnly(different_normed, 'red', 'different objects-false case')

    np.savetxt('similar_hausdor.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_hausdor.txt', different_normed, delimiter=',', fmt='%f')  # X is an array
    global similar_hausdorf_normed
    similar_hausdorf_normed = similar_normed
    global different_hausdorf_normed
    different_hausdorf_normed = different_normed
    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('HausdorfDistance')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def DataIndex():
    similar_normed, different_normed, t = normalize_my_data(Similar_DataIndex, Different_DataIndex, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar objets-true case')
    linesForOneMethodOnly(different_normed, 'red', 'different objects-false case')
    np.savetxt('similar_index.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_index.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_index_normed
    similar_index_normed = similar_normed
    global different_index_normed
    different_index_normed = different_normed

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('Data Index Max')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def DataNN():
    similar_normed, different_normed, t = normalize_my_data(Similar_DataNN, Different_DataNN, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar objets-true case')
    linesForOneMethodOnly(different_normed, 'red', 'different objects-false case')

    np.savetxt('similar_NN.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_NN.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_nn_normed
    similar_nn_normed = similar_normed
    global different_nn_normed
    different_nn_normed = different_normed


    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('DataNN')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def DataNNPlane():
    similar_normed, different_normed, t = normalize_my_data(Similar_DataNNPlane, Different_DataNNPlane, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar objets-true case')
    linesForOneMethodOnly(different_normed, 'red', 'different objects-false case')

    np.savetxt('similar_nnp.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_nnp.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_nnp_normed
    similar_nnp_normed = similar_normed
    global different_nnp_normed
    different_nnp_normed = different_normed

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('DataNNPlane')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def DataRIFT():
    similar_normed, different_normed, t = normalize_my_data(Similar_DataRift, Different_DataRift, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar objets-true case')
    linesForOneMethodOnly(different_normed, 'red', 'different objects-false case')

    np.savetxt('similar_rift.txt', similar_normed, delimiter=',', fmt='%f')  # X is an array
    np.savetxt('different_rift.txt', different_normed, delimiter=',', fmt='%f')  # X is an array

    global similar_rift_normed
    similar_rift_normed = similar_normed
    global different_rift_normed
    different_rift_normed = different_normed

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('RIFT')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()





def boxPlot():
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(9, 4))

    ####--------------------------------------------------------------------hausdorf
    bplot1 = axes.boxplot((similar_hausdorf_normed, different_hausdorf_normed),
                          positions=[0.85, 1.85],
                          widths=(0.03),
                          vert=True,  # vertical box alignment
                          patch_artist=True,
                          labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['mediumpurple', 'mediumpurple']
    for patch, color in zip(bplot1['boxes'], colors):
        patch.set_facecolor(color)

    mediumpurple_patch = mpatches.Patch(color='mediumpurple', label='hausdorf')
    ###--------------------------------------------------------------------nnp

    bplot2 = axes.boxplot(
        (similar_nnp_normed, different_nnp_normed),
        positions=[0.95, 1.95],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['orange', 'orange']
    for patch, color in zip(bplot2['boxes'], colors):
        patch.set_facecolor(color)

    orange_patch = mpatches.Patch(color='orange', label='nnp')

    ####--------------------------------------------------------------------nn
    bplot3 = axes.boxplot(
        (similar_nn_normed, different_nn_normed),
        positions=[1, 2],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['lightblue', 'lightblue']
    for patch, color in zip(bplot3['boxes'], colors):
        patch.set_facecolor(color)

    lightblue_patch = mpatches.Patch(color='lightblue', label='nn')

    ###--------------------------------------------------------------------index

    bplot4 = axes.boxplot(
        (similar_index_normed, different_index_normed),
        positions=[1.05, 2.05],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['lightgreen', 'lightgreen','lightgreen', 'lightgreen']
    for patch, color in zip(bplot4['boxes'], colors):
        patch.set_facecolor(color)

    lightgreen_patch = mpatches.Patch(color='lightgreen', label='index')

    ###--------------------------------------------------------------------rift

    bplot5 = axes.boxplot(
        (similar_rift_normed, different_rift_normed),
        positions=[1.10, 2.10],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['pink', 'pink','pink', 'pink']
    for patch, color in zip(bplot5['boxes'], colors):
        patch.set_facecolor(color)

    pink_patch = mpatches.Patch(color='pink', label='rift')

    ###--------------------------------------------------------------------nnp


    #plt.xlabel('Objects attribute compared')
    plt.xticks(x, labels)
    plt.ylabel('distance')

    plt.legend(handles=[mediumpurple_patch, orange_patch, lightblue_patch, lightgreen_patch, pink_patch])
    plt.show()


def boxPlot2():
    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(9, 4))

    ####--------------------------------------------------------------------hausdorf
    bplot1 = axes.boxplot((similar_shot_normed, different_shot_normed),
                          positions=[0.85, 1.85],
                          widths=(0.03),
                          vert=True,  # vertical box alignment
                          patch_artist=True,
                          labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['mediumpurple', 'mediumpurple']
    for patch, color in zip(bplot1['boxes'], colors):
        patch.set_facecolor(color)

    mediumpurple_patch = mpatches.Patch(color='mediumpurple', label='SHOTColor')
    ###--------------------------------------------------------------------nnp

    bplot2 = axes.boxplot(
        (similar_pfhrgb_normed, different_pfhrgb_normed),
        positions=[0.95, 1.95],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['orange', 'orange']
    for patch, color in zip(bplot2['boxes'], colors):
        patch.set_facecolor(color)

    orange_patch = mpatches.Patch(color='orange', label='pfhrgb')

    ####--------------------------------------------------------------------nn
    bplot3 = axes.boxplot(
        (similar_rsd_normed, different_rsd_normed),
        positions=[1, 2],
        widths=(0.03),
        vert=True,  # vertical box alignment
        patch_artist=True,
        labels=['', ''])  # will be used to label x-ticks

    # fill with colors
    colors = ['lightblue', 'lightblue']
    for patch, color in zip(bplot3['boxes'], colors):
        patch.set_facecolor(color)

    lightblue_patch = mpatches.Patch(color='lightblue', label='rsd')

    ###--------------------------------------------------------------------index


    #plt.xlabel('Objects attribute compared')
    plt.xticks(x, labels)
    plt.ylabel('correspondence')

    plt.legend(handles=[mediumpurple_patch, orange_patch, lightblue_patch])
    plt.show()



#print(cosine_similarity([similar_shot_normed], [different_shot_normed]))
#print(cosine_similarity([similar_rsd_normed], [different_rsd_normed]))
#print(cosine_similarity([similar_pfhrgb_normed], [different_pfhrgb_normed]))
#print(cosine_similarity([similar_pfhrgb_normed], [different_pfhrgb_normed]))
#boxPlot2()
#DataHausdorf()
#DataIndex()
# DataNN()
# DataNNPlane()
# DataRIFT()
#boxPlot()
#print(cosine_similarity([similar_hausdorf_normed], [different_hausdorf_normed]))

# ####--------------------------------------------------------------------pfh
#
# bplot3 = axes.boxplot(
#     (similar_pfh_normed, different_pfh_normed, shape_different_pfh_normed, texture_different_pfh_normed),
#     positions=[1.05, 2.05, 3.05, 4.05],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['', '', '', ''])  # will be used to label x-ticks
#
# # fill with colors
# colors = ['lightblue', 'lightblue', 'lightblue', 'lightblue']
# for patch, color in zip(bplot3['boxes'], colors):
#     patch.set_facecolor(color)
# lightblue_patch = mpatches.Patch(color='lightblue', label='PFH')
#
# ####--------------------------------------------------------------------pfhrgb
#
# bplot4 = axes.boxplot(
#     (similar_pfhrgb_normed, different_pfhrgb_normed, shape_different_pfhrgb_normed, texture_different_pfhrgb_normed),
#     positions=[1.10, 2.10, 3.10, 4.10],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['', '', '', ''])  # will be used to label x-ticks
#
# # fill with colors
# colors = ['blue', 'blue', 'blue', 'blue']
# for patch, color in zip(bplot4['boxes'], colors):
#     patch.set_facecolor(color)
# palegreen_patch = mpatches.Patch(color='blue', label='PFHRGB')
#
# ####--------------------------------------------------------------------RSD
#
# bplot5 = axes.boxplot(
#     (similar_RSD_normed, different_RSD_normed, shape_different_RSD_normed, texture_different_RSD_normed),
#     positions=[1.15, 2.15, 3.15, 4.15],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['', '', '', ''])  # will be used to label x-ticks
#
# # fill with colors
# colors = ['lightgreen', 'lightgreen', 'lightgreen', 'lightgreen']
# for patch, color in zip(bplot5['boxes'], colors):
#     patch.set_facecolor(color)
#
# lightgreen_patch = mpatches.Patch(color='lightgreen', label='RSD')
#
# ####--------------------------------------------------------------------rift
#
# bplot6 = axes.boxplot(
#     (similar_RIFT_normed, different_RIFT_normed, shape_different_RIFT_normed, texture_different_RIFT_normed),
#     positions=[1.20, 2.20, 3.20, 4.20],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['similar objects', 'different objects', 'shape different', 'texture different'])
# # will be used to label x-ticks
# axes.set_title('object similarity feature extraction match')
#
# # fill with colors
# colors = ['pink', 'pink', 'pink', 'pink']
# for patch, color in zip(bplot6['boxes'], colors):
#     patch.set_facecolor(color)
#
# pink_patch = mpatches.Patch(color='pink', label='RIFT')
#
# ####--------------------------------------------------------------------cvh
#
# bplot7 = axes.boxplot(
#     (similar_CVFH_normed, different_CVFH_normed, shape_different_CVFH_normed, texture_different_CVFH_normed),
#     positions=[1.25, 2.25, 3.25, 4.25],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['', '', '', ''])  # will be used to label x-ticks
#
# # fill with colors
# colors = ['yellow', 'yellow', 'yellow', 'yellow']
# for patch, color in zip(bplot7['boxes'], colors):
#     patch.set_facecolor(color)
#
# lightyellow_patch = mpatches.Patch(color='yellow', label='CVFH')
#
# ####--------------------------------------------------------------------harris
#
# bplot8 = axes.boxplot(
#     (similar_Harris_normed, different_Harris_normed, shape_different_Harris_normed, texture_different_Harris_normed),
#     positions=[1.30, 2.30, 3.30, 4.30],
#     widths=(0.02),
#     vert=True,  # vertical box alignment
#     patch_artist=True,
#     labels=['', '', '', ''])  # will be used to label x-ticks
#
# # fill with colors
# colors = ['slategrey', 'slategrey', 'slategrey', 'slategrey']
# for patch, color in zip(bplot8['boxes'], colors):
#     patch.set_facecolor(color)
#
# slategrey_patch = mpatches.Patch(color='slategrey', label='harris3dKeypoint')



def shotDP2():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_dp2, Diff_ShotColor_dp2,
                                                                      True)
    linesForOneMethodOnly(Similar_ShotColor_dp1, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_ShotColor_dp1, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('core/point2')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def shotmA():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_mALL, Diff_ShotColor_mALL,
                                                                      True)
    linesForOneMethodOnly(Similar_ShotColor_mALL, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_ShotColor_mALL, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('l Max All')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def shotmC():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_mC, Diff_ShotColor_mC,
                                                                      True)
    linesForOneMethodOnly(Similar_ShotColor_mC, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_ShotColor_mC, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('Max corre')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def shotmR():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_mR, Diff_ShotColor_mR,
                                                                      True)
    linesForOneMethodOnly(Similar_ShotColor_mR, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_ShotColor_mR, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('Max remaining')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


# ColorFit()
# pfh_max_corr()
# pfh_max_all()
# pfh_max_remain()
# pfh_aver_all()
# pfh_aver_corr()
# pfh_aver_upper()
#
# PFHRGB_max_corr()
# PFHRGB_max_all()
# PFHRGB_max_remain()
# PFHRGB_max_corr()
# PFHRGB_aver_all()
# PFHRGB_aver_corr()
# PFHRGB_aver_upper()
# PFHRGB_sqrt()
#
# ShapeContext_max_corr()
# ShapeContext_max_all()
# ShapeContext_max_remain()
# ShapeContext_aver_all()
# ShapeContext_aver_corr()
# ShapeContext_aver_upper()
#
# ShotColor_max_corr()
# ShotColor_max_all()
# ShotColor_max_remain()
# ShotColor_max_corr()
# ShotColor_maxWD()
# ShotColor_aver_all()
# ShotColor_aver_corr()
# ShotColor_aver_upper()
# ShotColor_sqrt()
# USC_max_corr()
# USC_max_all()
# USC_max_remain()
# USC_max_corr()
# USC_aver_all()
# USC_aver_corr()
# USC_aver_upper()
#
#
# rsd_max_corr()
# rsd_max_all()
# rsd_max_remain()
# rsd_max_corr()
# rsd_aver_all()
# rsd_aver_corr()
# rsd_aver_upper()
# rsd_sqrt()


def PFHRGB_max_all():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_max_all, Different_PFHRGB_max_all,
                                                                      True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max All')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_sqrt():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_sqrt, Different_PFHRGB_sqrt,
                                                                      True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.axhline(y=0.178, color='grey', linestyle='-')
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB sqrt')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_max_corr():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_max_corr,
                                                                      Different_PFHRGB_max_corr, False,
                                                                      Texture_PFHRGB_max_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max Correspondence')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_max_remain():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_max_remain_corr,
                                                                      Different_PFHRGB_max_remain_corr, True,
                                                                      Texture_PFHRGB_max_remain_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max Remaining Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_aver_all():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_average_all,
                                                                      Different_PFHRGB_average_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_aver_corr():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_average_corr,
                                                                      Different_PFHRGB_average_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def PFHRGB_aver_upper():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_average_upper,
                                                                      Different_PFHRGB_average_upper, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFHRGB Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_max_corr():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_max_corr,
                                                                      Different_ShotColor_max_corr, True)

    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    #
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max Correspondence')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_max_all():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_max_all,
                                                                      Different_ShotColor_max_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max All')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_max_remain():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_max_remain_corr,
                                                                      Different_ShotColor_max_remain_corr, True,
                                                                      Texture_ShotColor_max_remain_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    # plt.axhline(y=0.4, color='grey', linestyle='-')
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max Remaining Correspondence')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_aver_all():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_average_all,
                                                                      Different_ShotColor_average_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_aver_corr():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_average_corr,
                                                                      Different_ShotColor_average_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_aver_upper():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_average_upper,
                                                                      Different_ShotColor_average_upper, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_sqrt():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_sqrt,
                                                                      Different_ShotColor_sqrt, True,
                                                                      Texture_ShotColor_sqrt)
    linesForOneMethodOnly(Similar_ShotColor_sqrt, 'blue', 'trueCase')
    linesForOneMethodOnly(Different_ShotColor_sqrt, 'red', 'falseCase')

    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    # plt.axhline(y=0.365, color='grey', linestyle='-')
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor sqrt')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShotColor_maxWD():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ShotColor_maxWD,
                                                                      Different_ShotColor_maxWD, True)
    linesForOneMethodOnly(Similar_ShotColor_maxWD, 'blue', 'trueCase')
    linesForOneMethodOnly(Different_ShotColor_maxWD, 'red', 'falseCase')
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShotColor Max wd')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShotColor_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_max_corr():
    similar_normed, different_normed, t = normalize_my_data(Similar_PFH_max_corr, Different_PFH_max_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_max_all():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_max_all, Different_PFH_max_all)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_max_remain():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_max_remain_corr, Different_PFH_max_remain_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Remaining Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_aver_all():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_average_all, Different_PFH_average_all)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_aver_corr():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_average_corr, Different_PFH_average_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_aver_upper():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_average_upper, Different_PFH_average_upper)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfh_aver_upper():
    similar_normed, different_normed = normalize_my_data(Similar_PFH_average_upper, Different_PFH_average_upper)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('PFH Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_max_corr():
    similar_normed, different_normed = normalize_my_data(Similar_USC_max_corr, Different_USC_max_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_max_all():
    similar_normed, different_normed = normalize_my_data(Similar_USC_max_all, Different_USC_max_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_max_remain():
    similar_normed, different_normed = normalize_my_data(Similar_USC_max_remain_corr, Different_USC_max_remain_corr,
                                                         True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max Remaining Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_aver_all():
    similar_normed, different_normed = normalize_my_data(Similar_USC_average_all, Different_USC_average_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_aver_corr():
    similar_normed, different_normed = normalize_my_data(Similar_USC_average_corr, Different_USC_average_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def USC_aver_upper():
    similar_normed, different_normed = normalize_my_data(Similar_USC_average_upper, Different_USC_average_upper, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('USC Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/USC_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_max_corr():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_max_corr, Different_ShapeContext_max_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_max_all():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_max_all, Different_ShapeContext_max_all)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_max_remain():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_max_remain_corr,
                                                         Different_ShapeContext_max_remain_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max Remaining Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_aver_all():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_average_all,
                                                         Different_ShapeContext_average_all)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_aver_corr():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_average_corr,
                                                         Different_ShapeContext_average_corr)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def ShapeContext_aver_upper():
    similar_normed, different_normed = normalize_my_data(Similar_ShapeContext_average_upper,
                                                         Different_ShapeContext_average_upper)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('ShapeContext Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ShapeContext_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_max_corr():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_max_corr, Different_rsd_max_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_max_all():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_max_all, Different_rsd_max_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_max_remain():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_max_remain_corr,
                                                         Different_rsd_max_remain_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max Remaining Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_aver_all():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_average_all, Different_rsd_average_all, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max Average All')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_aver_corr():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_average_corr, Different_rsd_average_corr, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max Average Correspondence')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_aver_upper():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_average_upper, Different_rsd_average_upper, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd Max Average Upper')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFH_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def rsd_sqrt():
    similar_normed, different_normed = normalize_my_data(Similar_rsd_sqrt, Different_rsd_sqrt, True)
    linesForOneMethodOnly(similar_normed, 'blue', 'trueCase')
    linesForOneMethodOnly(different_normed, 'red', 'falseCase')

    n = [0, 1]
    plt.plot([0, 0], [0, 1], color='white')
    a = np.empty(59)
    b = [0.35]
    for i in a:
        b.append(0.35)
    y = b
    x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('rsd sqrt')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/rsd_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()




def pfhrgbDP2():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_dp2, Diff_PFHRGB_dp2,
                                                                      True)
    linesForOneMethodOnly(Similar_PFHRGB_dp1, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_PFHRGB_dp1, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('core/point2 Max All')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfhrgbmA():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_mALL, Diff_PFHRGB_mALL,
                                                                      True)
    linesForOneMethodOnly(Similar_PFHRGB_mALL, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_PFHRGB_mALL, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('l Max All')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfhrgbmC():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_mC, Diff_PFHRGB_mC,
                                                                      True)
    linesForOneMethodOnly(Similar_PFHRGB_mC, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_PFHRGB_mC, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('Max corre')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


def pfhrgbmR():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_PFHRGB_mR, Diff_PFHRGB_mR,
                                                                      True)
    linesForOneMethodOnly(Similar_PFHRGB_mR, 'blue', 'trueCase')
    linesForOneMethodOnly(Diff_PFHRGB_mR, 'red', 'falseCase')
    # linesForOneMethodOnly(Texture_PFHRGB_max_corr, 'magenta', 'miauw')
    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    plt.xlabel('iteration')

    plt.ylabel('distance')

    plt.title('Max remaining')

    plt.legend(loc='upper right')

    # plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/PFHRGB_max_all.png", bbox_inches='tight', dpi=1000)
    plt.show()


# shotDP1()
# shotDP2()
# shotmA()
# shotmC()
# shotmR()
#
#
# pfhrgbDP1()
# pfhrgbDP2()
# pfhrgbmA()
# pfhrgbmC()
# pfhrgbmR()
#
#


def ColorFit():
    similar_normed, different_normed, tNormedData = normalize_my_data(Similar_ColorFit_all, Different_ColorFit_all,
                                                                      True)
    linesForOneMethodOnly(similar_normed, 'blue', 'similar-object')
    linesForOneMethodOnly(different_normed, 'red', 'different-object')

    # n = [0, 1]
    # plt.plot([0, 0], [0, 1], color='white')
    # a = np.empty(59)
    # b = [0.35]
    # for i in a:
    #     b.append(0.35)
    # y = b
    # x = np.linspace(0, u, len(y))
    # plt.axhline(y=0.0210, color='grey', linestyle='-')
    plt.xlabel('iteration')

    plt.ylabel('correspondence')

    plt.title('ColorFit All Points')

    plt.legend(loc='upper right')

    plt.axis([0, u, 0, 1])
    # plt.savefig("Plots/ColorFit_all.png", bbox_inches='tight')
    plt.show()

