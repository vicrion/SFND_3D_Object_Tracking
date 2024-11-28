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
| LIDAR | - | `[12.52, 12.61, 14.09, 16.69, 15.91, 12.68, 11.98, 13.12, 13.02, 11.17, 12.81, 8.96, 9.96, 9.60, 8.57, 9.52, 9.55, 8.40, ]` | 
| BRISK | BRISK | `[13.70, 25.27, 16.41, 17.22, 21.82, 18.82, 15.68, 15.78, 15.46, 13.55, 11.06, 12.09, 12.29, 12.20, 12.79, 11.44, 9.35, 11.90, ]` | 
| BRISK | BRIEF | `[14.49, 12.34, 14.86, 17.78, 16.46, 16.67, 14.64, 14.40, 17.12, 11.56, 11.59, 13.25, 12.30, 11.39, 11.13, 9.60, 8.88, 10.41, ]` |
| BRISK | ORB | `[14.96, 23.20, 15.74, 16.69, 38.48, 27.38, 15.47, 17.37, 15.68, 12.72, 10.94, 13.29, 12.36, 12.66, 11.17, 11.57, 8.72, 12.17, ]` |
| BRISK | FREAK | `[13.82, 21.06, 13.10, 14.87, 23.05, 17.12, 19.55, 15.66, 16.61, 12.96, 11.97, 12.06, 12.05, 12.59, 12.60, 9.97, 8.61, 9.92, ]` |
| BRISK | AKAZE | - |
| BRISK | SIFT | `[15.76, 17.89, 18.39, 13.32, 34.55, 20.08, 15.07, 16.61, 17.21, 13.85, 13.63, 13.11, 14.78, 11.27, 14.76, 10.56, 9.66, 11.06, ]` |
| ORB | BRISK | `[16.73, -inf, 12.77, 18.06, 4008144.94, 18.12, -inf, 14.64, -inf, -inf, 8.51, -inf, 16.08, 20.05, 16.84, 12.30, 13.05, 36.31, ]` |
| ORB | BRIEF | `[20.18, -inf, 22.58, 14.13, 36.23, 10.13, -inf, -inf, -inf, 34.34, 24.59, 21.32, -inf, 20.17, 13.74, 9.37, 15.42, 16.81, ]` |
| ORB | ORB | `[12.18, -inf, 18.53, 30.38, -inf, -inf, -inf, -inf, -inf, 247.70, 8.02, -inf, 28.53, 38.83, 26.19, 16.84, 15.75, 245.09, ]` |
| ORB | FREAK | `[9.32, 47.92, 11.11, 11.37, -inf, 19.89, -inf, 9.53, -inf, -inf, 10.70, 22.56, 8.95, 54.18, 9.32, 7.31, 8.86, 8.82, ]` |
| ORB | AKAZE | - |
| ORB | SIFT | `[12.84, 20.89, 12.36, 44.29, -inf, -inf, -inf, 11.98, -inf, -inf, 8.29, -inf, 9.32, 17.72, 12.74, 9.50, 12.58, 24.87, ]` |
| AKAZE | BRISK | `[13.31, 13.92, 14.72, 14.98, 14.39, 15.34, 16.61, 14.11, 15.20, 13.20, 13.24, 11.15, 10.19, 9.80, 10.47, 11.77, 9.29, 9.02, ]` |
| AKAZE | BRIEF | `[13.59, 15.11, 13.06, 15.22, 15.06, 13.44, 16.18, 14.55, 14.24, 12.06, 12.77, 10.86, 10.31, 10.58, 10.07, 9.93, 9.51, 9.26, ]` |
| AKAZE | ORB | `[12.26, 13.50, 13.61, 14.18, 14.03, 13.77, 15.61, 14.11, 12.87, 12.38, 11.94, 10.81, 10.54, 12.76, 11.82, 10.61, 9.29, 8.43, ]` |
| AKAZE | FREAK | `[12.83, 13.81, 14.55, 15.53, 16.05, 16.29, 16.34, 13.45, 13.66, 12.08, 12.28, 11.23, 11.53, 10.07, 9.83, 9.61, 9.35, 9.07, ]` |
| AKAZE | AKAZE | `[12.45, 15.04, 13.06, 14.81, 14.70, 17.12, 15.74, 13.79, 14.61, 12.00, 12.27, 11.60, 11.08, 11.35, 10.00, 10.04, 9.42, 9.26, ]` |
| AKAZE | SIFT | `[12.66, 14.61, 13.66, 15.02, 15.16, 17.02, 15.41, 13.73, 14.52, 12.23, 12.24, 11.70, 10.70, 10.91, 10.93, 10.08, 9.18, 9.55, ]` |
| SIFT | BRISK | `[12.42, 13.11, 15.29, 17.56, 16.45, 11.86, 15.17, 14.70, 13.23, 11.66, 11.75, 10.62, 8.81, 9.13, 8.96, 8.36, 8.94, 9.02, ]` |
| SIFT | BRIEF | `[12.16, 14.82, 14.67, 17.77, 14.94, 13.74, 15.09, 14.89, 13.02, 11.52, 11.75, 11.11, 8.83, 9.48, 8.84, 8.13, 8.47, 9.91, ]` |
| SIFT | ORB | - |
| SIFT | FREAK | `[14.33, 13.33, 13.50, 19.29, 15.22, 11.74, 16.73, 15.24, 13.46, 12.17, 13.07, 10.45, 8.72, 9.65, 9.74, 8.34, 8.51, 10.51, ]` |
| SIFT | AKAZE | - |
| SIFT | SIFT | `[11.49, 12.87, 12.92, 17.11, 13.89, 12.19, 13.34, 14.18, 13.40, 10.66, 10.90, 10.62, 9.10, 9.22, 8.88, 8.44, 8.10, 7.99, ]` |
| FAST | BRISK | `[16.75, 11.94, 12.42, 14.97, 26.47, 13.50, 14.84, 14.20, 13.17, 15.42, 10.40, 12.65, 10.87, 11.39, 9.20, 9.85, 9.42, 9.54, ]` |
| FAST | BRIEF | `[12.22, 13.25, 12.34, 16.72, 16.08, 15.06, 16.99, 13.17, 14.49, 14.15, 11.69, 11.37, 10.20, 9.83, 10.70, 8.92, 9.27, 9.15, ]` |
| FAST | ORB | `[10.20, 12.45, 21.48, 13.79, 13.99, 13.37, 13.21, 13.15, 15.23, 15.02, 11.21, 11.98, 11.28, 10.04, 9.44, 9.27, 9.56, 10.33, ]` |
| FAST | FREAK | `[12.40, 12.37, 12.09, 15.49, 12.40, 12.27, 13.01, 12.35, 15.02, 13.30, 10.78, 12.45, 11.18, 11.06, 8.65, 9.57, 9.02, 9.16, ]` |
| FAST | AKAZE | - |
| FAST | SIFT | `[15.52, 11.38, 15.26, 38.28, 15.83, 13.41, 12.87, 13.99, 14.62, 16.55, 12.20, 11.87, 11.93, 10.91, 10.37, 9.93, 9.95, 10.25, ]` |


* The ratio is `1` signifies there will never be collision (TTC = `inf`).
* When compared to manual estimate (distance), ...

Performance summary:

* SIFT-SIFT provided the best performance among all the detectors-descriptors pairs.
* Other best performing pairs are: FAST-FREAK and most of AKAZE-* pairs.

[!plot](./performance.png)