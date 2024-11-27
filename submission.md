# Camera vs. Lidar TTC

## FP.5 : Performance Evaluation 1

Examples where the Lidar-based TTC estimate is way off - observations and argumentation.

All data observations:

```bash
LIDAR: median(prev)=8.074000, median(curr)=8.010000, TTC=12.515600  << [0] >>
LIDAR: median(prev)=7.947000, median(curr)=7.891000, TTC=14.091013  << [1] >>
LIDAR: median(prev)=7.891000, median(curr)=7.844000, TTC=16.689386  << [2] >>
LIDAR: median(prev)=7.844000, median(curr)=7.795000, TTC=15.908233  << [3] >>
LIDAR: median(prev)=7.795000, median(curr)=7.734000, TTC=12.678716
LIDAR: median(prev)=7.734000, median(curr)=7.670000, TTC=11.984351
LIDAR: median(prev)=7.670000, median(curr)=7.612000, TTC=13.124118
LIDAR: median(prev)=7.612000, median(curr)=7.554000, TTC=13.024118
LIDAR: median(prev)=7.554000, median(curr)=7.487000, TTC=11.174641
LIDAR: median(prev)=7.487000, median(curr)=7.429000, TTC=12.808601
LIDAR: median(prev)=7.429000, median(curr)=7.347000, TTC=8.959780
LIDAR: median(prev)=7.347000, median(curr)=7.274000, TTC=9.964390
LIDAR: median(prev)=7.274000, median(curr)=7.199000, TTC=9.598630
LIDAR: median(prev)=7.199000, median(curr)=7.116000, TTC=8.573525
LIDAR: median(prev)=7.116000, median(curr)=7.042000, TTC=9.516170
LIDAR: median(prev)=7.042000, median(curr)=6.969000, TTC=9.546581
LIDAR: median(prev)=6.969000, median(curr)=6.887000, TTC=8.398803
```

While most of the observations are rather close to each other (no immediate outliers), a few do not necessary make sense in comparison to the rest, such as `[1]`, `[2]` and `[3]` as marked above.

Outliers:

* Given the overall distance, the TTC of `[1-3]` data points should be smaller than the very first observation (marked as `[0]`).

Observations:

* `median()` function takes care of point clouds data outliers, so the average distance seems to be calculated correctly when compared to manual distance computation (from bird eye view LIDAR visualization).
* The possible explanation is likely to be related to the chosen **velocity model** which is an approximation to the relative velocity changes.
* Generally, LIDAR measurements present a reliable source for TTC estimation even when using the velocity model.

## FP.6 : Performance Evaluation 2

This last exercise is about running the different detector / descriptor combinations and looking at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons. This is the last task in the final project.

The task is complete once all detector / descriptor combinations implemented in previous chapters have been compared with regard to the TTC estimate on a frame-by-frame basis. To facilitate the comparison, a spreadsheet and graph should be used to represent the different TTCs.

All data observations:

| Detector | Descriptor | TTC |
| --- | --- | --- | 
| BRISK | BRISK | `[13.70, 25.27, 16.41, 17.22, 21.82, 18.82, 15.68, 15.78, 15.46, 13.55, 11.06, 12.09, 12.29, 12.20, 12.79, 11.44, 9.35, 11.90]` | 
| BRISK | BRIEF | 0 |
| BRISK | ORB | 0 |
| BRISK | FREAK | 0 |
| BRISK | AKAZE | 0 |
| BRISK | SIFT | 0 |