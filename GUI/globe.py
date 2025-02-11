"""
List of Latitudes and Longitudes to generate the map in the GNSS page.
Each row of the array is one continues line broken into line segments, generally representing each continent or island
Data is organized into [Longitudes,Latitudes], where positive is N and E and negative is S and W respectively.
"""

GLOBE = [[[-158.902025, 70.309486], [-159.227531, 58.341218], [-144.10025, 60.13877], [-132.139144, 53.419354], [-124.399604, 48.323387], [-123.69601, 37.557642], [-117.23107, 32.206991], [-110.107176, 22.790109], [-113.537199, 31.159934], [-97.866377, 15.016422], [-77.813935, 6.56366], [-80.98011, -7.288914], [-70.953888, -18.742507], [-74.823658, -53.391519], [-66.380524, -55.635748], [-69.546699, -51.789931], [-56.530201, -36.396968], [-34.542874, -6.416295], [-72.203606, 12.111837], [-83.812915, 15.525145], [-94.190933, 17.714677], [-97.357108, 23.147937], [-96.061402, 28.55256], [-84.060693, 29.86774], [-81.532512, 24.800448], [-79.907729, 26.262323], [-81.290906, 31.256835], [-75.574758, 35.511549], [-70.080714, 41.357742], [-59.74667, 46.116562], [-64.671831, 49.02121], [-68.239999, 48.353968], [-65.909342, 50.301183], [-59.928789, 49.991408], [-55.707222, 52.117891], [-64.291689, 60.189329], [-61.65321, 66.75183], [-77.308187, 73.599119], [-61.829108, 82.412222], [-77.132288, 82.906151], [-98.76688, 80.36941], [-118.115728, 77.251449], [-128.493746, 69.560431], [-159.803701, 70.753439]],
         [[-43.19477, 59.393374], [-22.262834, 70.105812], [-10.846208, 81.477258], [-34.770768, 83.665741], [-60.451966, 81.784248], [-71.709478, 77.520155], [-62.210952, 75.948901], [-54.804431, 71.324553], [-43.19477, 59.393374]],
         [[-23.356371, 66.033084], [-15.001187, 66.175493], [-14.033744, 64.336934], [-21.333537, 63.289853], [-23.356371, 66.033084]],
         [[-79.1184, 43.203925], [-79.536159, 43.603019], [-76.435946, 43.967863], [-76.347997, 43.411782], [-78.986476, 43.139826], [-78.920514, 42.802202], [-80.899374, 41.794608], [-83.23003, 41.466142], [-83.210317, 42.002047], [-80.482443, 42.564587], [-78.954324, 42.84715]],
         [[-82.383621, 42.98393], [-81.229286, 44.648506], [-80.152905, 44.515481], [-80.735569, 45.824373], [-83.879757, 46.228659], [-84.75925, 45.908525], [-86.683303, 45.497444], [-87.716707, 44.188851], [-87.69472, 41.827363], [-87.199205, 41.594263], [-86.209775, 42.621202], [-86.517598, 43.829945], [-84.912523, 45.709416], [-83.371261, 45.262684], [-83.942931, 43.710944], [-83.635109, 43.56783], [-82.832571, 44.027754], [-82.392825, 42.98393]],
         [[-84.558958, 46.471325], [-85.009698, 47.963606], [-86.032109, 47.948891], [-86.482849, 48.701271], [-88.965393, 48.22776], [-92.054612, 46.735508], [-90.746366, 46.622444], [-88.525646, 46.975919], [-87.283362, 46.478891], [-85.073636, 46.712914], [-84.611902, 46.43348]],
         [[-65.098491, 59.863436], [-77.763778, 62.244269], [-79.083018, 50.759916], [-82.425092, 54.796727], [-92.011567, 57.013076], [-94.562097, 59.907533], [-85.497038, 65.997357], [-80.747776, 67.013328], [-81.011624, 69.730956], [-72.392591, 67.286364], [-77.845448, 65.12475], [-75.558766, 64.030737], [-64.125356, 61.372384]],
         [[-76.331731, 44.157332], [-73.693252, 45.497444], [-70.636265, 47.001563], [-69.075165, 48.230848]],
         [[-5.124328, 34.965199], [-12.614794, 27.441985], [-17.538106, 21.902273], [-16.957858, 13.368241], [-7.269485, 4.013311], [5.812458, 5.572244], [8.59061, -1.13372], [13.81284, -11.961946], [11.720433, -16.897068], [18.261404, -33.790104], [25.482658, -34.415971], [35.593031, -24.599075], [40.621844, -14.987237], [39.144851, -4.311359], [51.24919, 11.70388], [44.305677, 10.064057], [32.344918, 31.019048], [21.302631, 32.890661], [10.072897, 33.673153], [10.682342, 37.023607], [-2.72221, 35.105529], [-6.625693, 34.165455]],
         [[46.955015, -25.304303], [50.38375, -15.347761], [48.782505, -12.421556], [44.38669, -16.505619], [44.500982, -19.84526], [43.270154, -22.224022], [43.797651, -24.88245], [46.955015, -25.304303]],
         [[-8.937596, 36.904225], [-2.124084, 36.693092], [8.847869, 44.355281], [16.021839, 38.983327], [15.538299, 37.673389], [18.219746, 40.135213], [12.044159, 44.374128], [13.582091, 45.401536], [19.366982, 41.787697], [22.619884, 36.191092], [29.35987, 41.08929], [28.786541, 44.562295], [33.490062, 45.801233], [41.57836, 41.846649], [40.170558, 40.916834], [34.67579, 41.938246], [29.576645, 41.314125], [27.739194, 36.191092], [35.775693, 36.56613], [34.500907, 31.35739], [32.637083, 31.026577], [34.219576, 27.617352], [35.010823, 28.083612], [43.580799, 12.558853], [60.04317, 22.256564], [55.735272, 25.542443], [66.64717, 24.950205], [72.449645, 19.584635], [77.302625, 7.615178], [88.169081, 21.722766], [104.961217, 8.311515], [109.487242, 12.481633], [105.654092, 19.157623], [116.759588, 22.893641], [122.276381, 29.867326], [119.577402, 34.564479], [122.346842, 36.953404], [121.863303, 38.969659], [124.896415, 39.37847], [126.610782, 34.245411], [129.248271, 35.757211], [127.577669, 39.507432], [134.646255, 42.834085], [140.501135, 48.496223], [141.468214, 53.364976], [135.555719, 54.701773], [142.975853, 59.362316], [170.986632, 59.932995], [179.607366, 62.392406], [187.607748, 64.303728], [190.315274, 66.034089], [179.29957, 69.101507], [170.868398, 70.058092], [159.957988, 69.692193], [158.121522, 70.905144], [140.631082, 72.766669], [131.329537, 71.094002], [113.735781, 73.851731], [104.856236, 77.619594], [80.345581, 72.821267], [76.37934, 71.882212], [72.730814, 71.648449], [72.159358, 72.710597], [69.214162, 72.658279], [66.629793, 69.040286], [44.299056, 66.418507], [24.869557, 71.139514], [17.396672, 69.727245], [4.621446, 61.627289], [7.276519, 57.877483], [11.108898, 58.920531], [13.702429, 55.296634], [16.559708, 56.260441], [19.065322, 59.816106], [17.273574, 61.965814], [22.900217, 65.699903], [24.745638, 65.623837], [24.306056, 64.493618], [20.877321, 62.716477], [21.070737, 60.119619], [23.119185, 59.740898], [22.196064, 57.570067], [20.965236, 54.909462], [11.470277, 54.056808], [5.113929, 53.254698], [1.518154, 50.560908], [-5.163484, 48.391278], [-1.446872, 46.289261], [-1.886453, 43.456109], [-8.963715, 43.264406], [-8.916703, 37.058684], [-5.619842, 36.212369]],
         [[-6.062426, 49.755719], [1.14671, 51.044157], [-2.572149, 56.177362], [-1.665688, 57.541775], [-5.534004, 58.633504], [-6.325251, 56.513442], [-5.133998, 54.925248], [-5.650506, 54.773445], [-6.63023, 55.170084], [-8.05887, 55.24531], [-8.652305, 54.524907], [-10.146882, 54.269073], [-9.513884, 52.734961], [-10.293875, 51.814728], [-9.489441, 51.416338], [-6.40673, 52.172584], [-6.3654, 53.950267], [-5.400518, 54.44002], [-4.714771, 54.684629], [-3.899348, 54.846888], [-2.998206, 53.4887], [-4.512564, 53.347273], [-4.048805, 52.373587], [-5.316603, 51.859532], [-3.487944, 51.383438], [-4.103358, 51.197935], [-5.764976, 50.015505], [-6.062426, 49.755719]],
         [[95.433563, 4.591756], [105.069313, -7.684861], [124.19923, -9.353681], [106.791806, -5.432273], [102.220159, 0.536125], [94.694525, 5.922045]],
         [[108.954545, 1.590617], [111.328285, -3.680083], [116.163681, -4.381464], [118.859144, 0.619618], [116.83707, 6.887164], [109.01252, 1.805854]],
         [[113.53215, -23.257493], [115.502344, -35.732248], [131.151443, -31.638421], [140.294737, -38.120732], [149.60075, -37.989238], [153.732816, -28.141755], [146.327597, -19.369193], [142.518215, -11.001589], [140.452183, -17.639833], [135.528871, -15.237548], [136.759577, -11.929702], [131.04641, -11.305552], [129.107857, -14.878963], [126.558284, -13.835946], [121.173618, -19.539082], [113.832608, -22.044911], [113.53215, -23.257493]],
         [[129.819791, 34.023527], [138.171838, 38.072312], [141.600573, 45.589441], [145.504059, 43.675818], [142.976305, 41.95132], [141.042147, 42.114524], [141.833393, 38.565348], [140.787189, 35.478564], [135.88146, 33.200087], [131.925227, 32.090023], [131.397729, 30.890914], [129.199822, 31.567305], [129.819791, 34.023527]],
         [[166.502051, -46.057984], [174.2211, -38.764362], [172.726523, -34.281728], [178.20358, -38.027324], [175.477726, -41.768034], [169.402711, -46.809085], [166.502051, -46.057984]],
         [[-180.0, -78.054715], [-161.543699, -78.623073], [-122.860532, -73.91273], [-77.854271, -73.216551], [-60.705847, -65.027643], [-62.112508, -75.293275], [-38.902608, -78.256756], [-9.292398, -71.954502], [52.76866, -66.499245], [75.767558, -69.791098], [135.02751, -65.932138], [171.319347, -71.801411], [162.176053, -77.730283], [180.0, -78.170583]]
         ]

