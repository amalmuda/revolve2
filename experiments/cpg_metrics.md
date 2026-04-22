# CPG Metrics Analysis

Per-run frequency (Welch peak) and Kuramoto order parameter R, computed by
offline RK45 integration of saved CMA-ES best parameters (no physics).

- Simulation time: 30 s, control frequency 20 Hz (600 samples per neuron)
- Transient skipped for Kuramoto R: first 5 seconds (100 samples)
- Frequency: peak of Welch PSD per hinge, then mean across all hinges
- Kuramoto R: mean of `|mean(exp(i*phase))|` over time, using Hilbert phases of inter-limb hinges
- Inter-limb hinges: spider = 4 hips; gecko = front L/R hips + rear L/R hips + spine 1 + spine 2

## Summary tables (mean ± std, 30 runs)

### Spider

| Coupling | λ | Distance (m) | Frequency (Hz) | Kuramoto R |
|---|---|---|---|---|
| No coupling | 0 | 1.70 ± 0.18 | 0.138 ± 0.010 | 0.424 ± 0.076 |
| No coupling | 1 | 1.51 ± 0.26 | 0.136 ± 0.015 | 0.475 ± 0.064 |
| No coupling | 2 | 1.39 ± 0.26 | 0.132 ± 0.011 | 0.456 ± 0.091 |
| No coupling | 3 | 1.20 ± 0.26 | 0.118 ± 0.013 | 0.469 ± 0.079 |
| Neighbour | 0 | 2.71 ± 0.14 | 0.211 ± 0.019 | 0.337 ± 0.076 |
| Neighbour | 1 | 2.42 ± 0.23 | 0.202 ± 0.020 | 0.369 ± 0.071 |
| Neighbour | 2 | 2.09 ± 0.25 | 0.197 ± 0.027 | 0.434 ± 0.069 |
| Neighbour | 3 | 1.92 ± 0.25 | 0.195 ± 0.026 | 0.450 ± 0.070 |
| Structured | 0 | 2.97 ± 0.23 | 0.289 ± 0.042 | 0.501 ± 0.094 |
| Structured | 1 | 2.97 ± 0.67 | 0.295 ± 0.060 | 0.513 ± 0.082 |
| Structured | 2 | 3.00 ± 0.81 | 0.307 ± 0.068 | 0.537 ± 0.064 |
| Structured | 3 | 2.96 ± 0.84 | 0.317 ± 0.065 | 0.531 ± 0.077 |

### Gecko

| Coupling | λ | Distance (m) | Frequency (Hz) | Kuramoto R |
|---|---|---|---|---|
| No coupling | 0 | 1.76 ± 0.28 | 0.142 ± 0.008 | 0.469 ± 0.080 |
| No coupling | 1 | 1.71 ± 0.27 | 0.142 ± 0.012 | 0.483 ± 0.075 |
| No coupling | 2 | 1.47 ± 0.32 | 0.130 ± 0.013 | 0.486 ± 0.052 |
| No coupling | 3 | 1.30 ± 0.41 | 0.121 ± 0.020 | 0.495 ± 0.054 |
| Neighbour | 0 | 1.60 ± 0.26 | 0.226 ± 0.023 | 0.406 ± 0.057 |
| Neighbour | 1 | 1.84 ± 0.34 | 0.229 ± 0.023 | 0.449 ± 0.043 |
| Neighbour | 2 | 1.93 ± 0.34 | 0.224 ± 0.018 | 0.482 ± 0.033 |
| Neighbour | 3 | 1.83 ± 0.41 | 0.225 ± 0.018 | 0.481 ± 0.033 |
| Structured | 0 | 4.47 ± 0.55 | 0.401 ± 0.047 | 0.608 ± 0.100 |
| Structured | 1 | 4.34 ± 0.69 | 0.396 ± 0.028 | 0.595 ± 0.113 |
| Structured | 2 | 3.92 ± 0.67 | 0.391 ± 0.040 | 0.545 ± 0.110 |
| Structured | 3 | 3.97 ± 0.67 | 0.375 ± 0.047 | 0.541 ± 0.117 |

## Per-run data

| morphology | coupling | lambda | run_id | distance | frequency_hz | kuramoto_R |
|---|---|---|---|---|---|---|
| Spider | No coupling | 0 | 1 | 1.8371 | 0.1367 | 0.4160 |
| Spider | No coupling | 0 | 2 | 1.3795 | 0.1465 | 0.3414 |
| Spider | No coupling | 0 | 3 | 1.8190 | 0.1562 | 0.4919 |
| Spider | No coupling | 0 | 4 | 1.6695 | 0.1367 | 0.4488 |
| Spider | No coupling | 0 | 5 | 1.7864 | 0.1270 | 0.3808 |
| Spider | No coupling | 0 | 6 | 1.7312 | 0.1465 | 0.3642 |
| Spider | No coupling | 0 | 7 | 1.6411 | 0.1367 | 0.4119 |
| Spider | No coupling | 0 | 8 | 2.0738 | 0.1465 | 0.4529 |
| Spider | No coupling | 0 | 9 | 1.6235 | 0.1367 | 0.4540 |
| Spider | No coupling | 0 | 10 | 1.9858 | 0.1367 | 0.4223 |
| Spider | No coupling | 0 | 11 | 1.5461 | 0.1172 | 0.3011 |
| Spider | No coupling | 0 | 12 | 1.7387 | 0.1367 | 0.4477 |
| Spider | No coupling | 0 | 13 | 1.6525 | 0.1465 | 0.4019 |
| Spider | No coupling | 0 | 14 | 1.6871 | 0.1367 | 0.4894 |
| Spider | No coupling | 0 | 15 | 1.7591 | 0.1367 | 0.3554 |
| Spider | No coupling | 0 | 16 | 1.7387 | 0.1367 | 0.3857 |
| Spider | No coupling | 0 | 17 | 1.5637 | 0.1270 | 0.3980 |
| Spider | No coupling | 0 | 18 | 1.8876 | 0.1465 | 0.3051 |
| Spider | No coupling | 0 | 19 | 1.2501 | 0.1172 | 0.4948 |
| Spider | No coupling | 0 | 20 | 1.6338 | 0.1367 | 0.3800 |
| Spider | No coupling | 0 | 21 | 1.6059 | 0.1367 | 0.5385 |
| Spider | No coupling | 0 | 22 | 1.7812 | 0.1562 | 0.3445 |
| Spider | No coupling | 0 | 23 | 1.8375 | 0.1465 | 0.2900 |
| Spider | No coupling | 0 | 24 | 1.7224 | 0.1367 | 0.5373 |
| Spider | No coupling | 0 | 25 | 1.4558 | 0.1367 | 0.5712 |
| Spider | No coupling | 0 | 26 | 1.7861 | 0.1465 | 0.3611 |
| Spider | No coupling | 0 | 27 | 2.0270 | 0.1367 | 0.5044 |
| Spider | No coupling | 0 | 28 | 1.4410 | 0.1172 | 0.5680 |
| Spider | No coupling | 0 | 29 | 1.6083 | 0.1367 | 0.3862 |
| Spider | No coupling | 0 | 30 | 1.6381 | 0.1465 | 0.4623 |
| Spider | No coupling | 1 | 1 | 1.4613 | 0.1270 | 0.3554 |
| Spider | No coupling | 1 | 2 | 1.3100 | 0.1367 | 0.4905 |
| Spider | No coupling | 1 | 3 | 2.0057 | 0.1465 | 0.4699 |
| Spider | No coupling | 1 | 4 | 1.2925 | 0.1270 | 0.3877 |
| Spider | No coupling | 1 | 5 | 1.9315 | 0.1562 | 0.3605 |
| Spider | No coupling | 1 | 6 | 1.5321 | 0.1270 | 0.3960 |
| Spider | No coupling | 1 | 7 | 1.3621 | 0.1172 | 0.5906 |
| Spider | No coupling | 1 | 8 | 1.8073 | 0.1562 | 0.5518 |
| Spider | No coupling | 1 | 9 | 1.3506 | 0.1367 | 0.5622 |
| Spider | No coupling | 1 | 10 | 1.2358 | 0.1270 | 0.4230 |
| Spider | No coupling | 1 | 11 | 1.6903 | 0.1465 | 0.4440 |
| Spider | No coupling | 1 | 12 | 1.2602 | 0.1172 | 0.5198 |
| Spider | No coupling | 1 | 13 | 1.7216 | 0.1172 | 0.4725 |
| Spider | No coupling | 1 | 14 | 1.5878 | 0.1367 | 0.5133 |
| Spider | No coupling | 1 | 15 | 1.6158 | 0.1367 | 0.5526 |
| Spider | No coupling | 1 | 16 | 1.2323 | 0.1172 | 0.5792 |
| Spider | No coupling | 1 | 17 | 1.0211 | 0.1074 | 0.4519 |
| Spider | No coupling | 1 | 18 | 1.6050 | 0.1562 | 0.5169 |
| Spider | No coupling | 1 | 19 | 1.3585 | 0.1270 | 0.5027 |
| Spider | No coupling | 1 | 20 | 1.6551 | 0.1562 | 0.4490 |
| Spider | No coupling | 1 | 21 | 1.5797 | 0.1562 | 0.4542 |
| Spider | No coupling | 1 | 22 | 1.4182 | 0.1270 | 0.5023 |
| Spider | No coupling | 1 | 23 | 1.9844 | 0.1465 | 0.4774 |
| Spider | No coupling | 1 | 24 | 1.0874 | 0.1172 | 0.5669 |
| Spider | No coupling | 1 | 25 | 1.7453 | 0.1562 | 0.4588 |
| Spider | No coupling | 1 | 26 | 1.2525 | 0.1270 | 0.3960 |
| Spider | No coupling | 1 | 27 | 1.7627 | 0.1562 | 0.4593 |
| Spider | No coupling | 1 | 28 | 1.5228 | 0.1465 | 0.4650 |
| Spider | No coupling | 1 | 29 | 1.2787 | 0.1367 | 0.5001 |
| Spider | No coupling | 1 | 30 | 1.7427 | 0.1367 | 0.3845 |
| Spider | No coupling | 2 | 1 | 1.1633 | 0.1172 | 0.2790 |
| Spider | No coupling | 2 | 2 | 1.4142 | 0.1270 | 0.4356 |
| Spider | No coupling | 2 | 3 | 1.1772 | 0.1172 | 0.5845 |
| Spider | No coupling | 2 | 4 | 1.4569 | 0.1367 | 0.4644 |
| Spider | No coupling | 2 | 5 | 1.5630 | 0.1270 | 0.6232 |
| Spider | No coupling | 2 | 6 | 1.3565 | 0.1270 | 0.4666 |
| Spider | No coupling | 2 | 7 | 1.2081 | 0.1367 | 0.5734 |
| Spider | No coupling | 2 | 8 | 1.5008 | 0.1367 | 0.3649 |
| Spider | No coupling | 2 | 9 | 1.1802 | 0.1367 | 0.4738 |
| Spider | No coupling | 2 | 10 | 1.6023 | 0.1367 | 0.4854 |
| Spider | No coupling | 2 | 11 | 0.8572 | 0.1172 | 0.4929 |
| Spider | No coupling | 2 | 12 | 1.5267 | 0.1270 | 0.3858 |
| Spider | No coupling | 2 | 13 | 0.7557 | 0.1074 | 0.5247 |
| Spider | No coupling | 2 | 14 | 1.5404 | 0.1270 | 0.4384 |
| Spider | No coupling | 2 | 15 | 1.6025 | 0.1465 | 0.3804 |
| Spider | No coupling | 2 | 16 | 1.5894 | 0.1367 | 0.6142 |
| Spider | No coupling | 2 | 17 | 1.2717 | 0.1367 | 0.3107 |
| Spider | No coupling | 2 | 18 | 1.4159 | 0.1465 | 0.3829 |
| Spider | No coupling | 2 | 19 | 1.6663 | 0.1465 | 0.3705 |
| Spider | No coupling | 2 | 20 | 1.7847 | 0.1465 | 0.3339 |
| Spider | No coupling | 2 | 21 | 1.5954 | 0.1270 | 0.3677 |
| Spider | No coupling | 2 | 22 | 1.4930 | 0.1270 | 0.4104 |
| Spider | No coupling | 2 | 23 | 1.1525 | 0.1270 | 0.3663 |
| Spider | No coupling | 2 | 24 | 1.3748 | 0.1465 | 0.5716 |
| Spider | No coupling | 2 | 25 | 1.8474 | 0.1367 | 0.5144 |
| Spider | No coupling | 2 | 26 | 1.0663 | 0.1172 | 0.4744 |
| Spider | No coupling | 2 | 27 | 1.6844 | 0.1465 | 0.5503 |
| Spider | No coupling | 2 | 28 | 1.3086 | 0.1367 | 0.4359 |
| Spider | No coupling | 2 | 29 | 0.9657 | 0.1172 | 0.5655 |
| Spider | No coupling | 2 | 30 | 1.5365 | 0.1367 | 0.4346 |
| Spider | No coupling | 3 | 1 | 1.4568 | 0.1367 | 0.5169 |
| Spider | No coupling | 3 | 2 | 0.9583 | 0.1367 | 0.4793 |
| Spider | No coupling | 3 | 3 | 1.3309 | 0.1172 | 0.4741 |
| Spider | No coupling | 3 | 4 | 1.3630 | 0.1172 | 0.4160 |
| Spider | No coupling | 3 | 5 | 1.0761 | 0.1074 | 0.5571 |
| Spider | No coupling | 3 | 6 | 1.3982 | 0.1367 | 0.3278 |
| Spider | No coupling | 3 | 7 | 1.3131 | 0.1172 | 0.6661 |
| Spider | No coupling | 3 | 8 | 1.5954 | 0.1367 | 0.5958 |
| Spider | No coupling | 3 | 9 | 1.7025 | 0.1367 | 0.5412 |
| Spider | No coupling | 3 | 10 | 1.1111 | 0.1074 | 0.3975 |
| Spider | No coupling | 3 | 11 | 1.1211 | 0.1172 | 0.4292 |
| Spider | No coupling | 3 | 12 | 1.2890 | 0.1270 | 0.4252 |
| Spider | No coupling | 3 | 13 | 1.1576 | 0.1074 | 0.4999 |
| Spider | No coupling | 3 | 14 | 0.5836 | 0.0781 | 0.3463 |
| Spider | No coupling | 3 | 15 | 1.3791 | 0.1270 | 0.4594 |
| Spider | No coupling | 3 | 16 | 0.7406 | 0.0977 | 0.4097 |
| Spider | No coupling | 3 | 17 | 1.2741 | 0.1172 | 0.3581 |
| Spider | No coupling | 3 | 18 | 1.1715 | 0.1172 | 0.5162 |
| Spider | No coupling | 3 | 19 | 1.7392 | 0.1270 | 0.4931 |
| Spider | No coupling | 3 | 20 | 0.8874 | 0.1074 | 0.3887 |
| Spider | No coupling | 3 | 21 | 1.3105 | 0.1270 | 0.5289 |
| Spider | No coupling | 3 | 22 | 0.8080 | 0.0977 | 0.4619 |
| Spider | No coupling | 3 | 23 | 1.1968 | 0.1172 | 0.5550 |
| Spider | No coupling | 3 | 24 | 1.4138 | 0.1270 | 0.4490 |
| Spider | No coupling | 3 | 25 | 1.0592 | 0.1172 | 0.5236 |
| Spider | No coupling | 3 | 26 | 0.8583 | 0.1074 | 0.5561 |
| Spider | No coupling | 3 | 27 | 1.0533 | 0.1172 | 0.5070 |
| Spider | No coupling | 3 | 28 | 1.2649 | 0.1172 | 0.3639 |
| Spider | No coupling | 3 | 29 | 1.1222 | 0.1270 | 0.4482 |
| Spider | No coupling | 3 | 30 | 1.2082 | 0.1172 | 0.3708 |
| Spider | Neighbour | 0 | 1 | 2.7244 | 0.2148 | 0.4322 |
| Spider | Neighbour | 0 | 2 | 2.8158 | 0.1953 | 0.3124 |
| Spider | Neighbour | 0 | 3 | 2.6070 | 0.1855 | 0.3663 |
| Spider | Neighbour | 0 | 4 | 2.4064 | 0.1758 | 0.3626 |
| Spider | Neighbour | 0 | 5 | 2.5717 | 0.1953 | 0.3857 |
| Spider | Neighbour | 0 | 6 | 2.7154 | 0.2051 | 0.3428 |
| Spider | Neighbour | 0 | 7 | 2.7335 | 0.1953 | 0.2617 |
| Spider | Neighbour | 0 | 8 | 2.5657 | 0.2148 | 0.2631 |
| Spider | Neighbour | 0 | 9 | 2.6985 | 0.2344 | 0.3589 |
| Spider | Neighbour | 0 | 10 | 2.6453 | 0.2344 | 0.4228 |
| Spider | Neighbour | 0 | 11 | 2.9458 | 0.1953 | 0.3808 |
| Spider | Neighbour | 0 | 12 | 2.7700 | 0.2344 | 0.3049 |
| Spider | Neighbour | 0 | 13 | 2.8406 | 0.2344 | 0.3264 |
| Spider | Neighbour | 0 | 14 | 2.7116 | 0.1953 | 0.1772 |
| Spider | Neighbour | 0 | 15 | 2.6915 | 0.1953 | 0.2622 |
| Spider | Neighbour | 0 | 16 | 2.5663 | 0.2148 | 0.3873 |
| Spider | Neighbour | 0 | 17 | 2.9628 | 0.1953 | 0.3671 |
| Spider | Neighbour | 0 | 18 | 2.3957 | 0.2148 | 0.5086 |
| Spider | Neighbour | 0 | 19 | 2.8533 | 0.2344 | 0.3021 |
| Spider | Neighbour | 0 | 20 | 2.7531 | 0.2344 | 0.4016 |
| Spider | Neighbour | 0 | 21 | 2.6462 | 0.2148 | 0.4081 |
| Spider | Neighbour | 0 | 22 | 2.8788 | 0.2344 | 0.3463 |
| Spider | Neighbour | 0 | 23 | 2.6963 | 0.1953 | 0.1691 |
| Spider | Neighbour | 0 | 24 | 2.7129 | 0.1953 | 0.4289 |
| Spider | Neighbour | 0 | 25 | 2.8071 | 0.1953 | 0.3261 |
| Spider | Neighbour | 0 | 26 | 2.7143 | 0.1855 | 0.3322 |
| Spider | Neighbour | 0 | 27 | 2.8353 | 0.2344 | 0.3490 |
| Spider | Neighbour | 0 | 28 | 2.8399 | 0.2344 | 0.2771 |
| Spider | Neighbour | 0 | 29 | 2.6232 | 0.2344 | 0.1962 |
| Spider | Neighbour | 0 | 30 | 2.4879 | 0.2148 | 0.3636 |
| Spider | Neighbour | 1 | 1 | 2.5018 | 0.1758 | 0.3570 |
| Spider | Neighbour | 1 | 2 | 2.4938 | 0.2148 | 0.4222 |
| Spider | Neighbour | 1 | 3 | 2.1381 | 0.2148 | 0.3559 |
| Spider | Neighbour | 1 | 4 | 2.1391 | 0.1953 | 0.4819 |
| Spider | Neighbour | 1 | 5 | 2.6568 | 0.1953 | 0.4165 |
| Spider | Neighbour | 1 | 6 | 2.5540 | 0.1953 | 0.3610 |
| Spider | Neighbour | 1 | 7 | 2.7276 | 0.2344 | 0.3736 |
| Spider | Neighbour | 1 | 8 | 1.9423 | 0.1953 | 0.5315 |
| Spider | Neighbour | 1 | 9 | 2.2557 | 0.1953 | 0.3661 |
| Spider | Neighbour | 1 | 10 | 2.4323 | 0.2344 | 0.3426 |
| Spider | Neighbour | 1 | 11 | 2.0778 | 0.2148 | 0.3732 |
| Spider | Neighbour | 1 | 12 | 2.4584 | 0.2344 | 0.4107 |
| Spider | Neighbour | 1 | 13 | 2.2314 | 0.1855 | 0.3943 |
| Spider | Neighbour | 1 | 14 | 2.4672 | 0.1953 | 0.3117 |
| Spider | Neighbour | 1 | 15 | 2.6502 | 0.2344 | 0.4095 |
| Spider | Neighbour | 1 | 16 | 2.4696 | 0.1855 | 0.4436 |
| Spider | Neighbour | 1 | 17 | 2.2525 | 0.2344 | 0.4498 |
| Spider | Neighbour | 1 | 18 | 2.7289 | 0.1562 | 0.4327 |
| Spider | Neighbour | 1 | 19 | 2.3566 | 0.1953 | 0.2692 |
| Spider | Neighbour | 1 | 20 | 1.9540 | 0.1953 | 0.2884 |
| Spider | Neighbour | 1 | 21 | 2.4859 | 0.2344 | 0.2397 |
| Spider | Neighbour | 1 | 22 | 2.8214 | 0.1953 | 0.4202 |
| Spider | Neighbour | 1 | 23 | 2.4338 | 0.1953 | 0.3286 |
| Spider | Neighbour | 1 | 24 | 2.5545 | 0.1953 | 0.3195 |
| Spider | Neighbour | 1 | 25 | 2.7123 | 0.1953 | 0.1926 |
| Spider | Neighbour | 1 | 26 | 2.4229 | 0.2051 | 0.4099 |
| Spider | Neighbour | 1 | 27 | 2.1116 | 0.1758 | 0.3409 |
| Spider | Neighbour | 1 | 28 | 2.3233 | 0.1953 | 0.3484 |
| Spider | Neighbour | 1 | 29 | 2.4972 | 0.1953 | 0.4023 |
| Spider | Neighbour | 1 | 30 | 2.6213 | 0.1953 | 0.2738 |
| Spider | Neighbour | 2 | 1 | 2.0323 | 0.1953 | 0.4984 |
| Spider | Neighbour | 2 | 2 | 2.0194 | 0.1953 | 0.4454 |
| Spider | Neighbour | 2 | 3 | 2.3575 | 0.1758 | 0.5104 |
| Spider | Neighbour | 2 | 4 | 2.1671 | 0.1953 | 0.3903 |
| Spider | Neighbour | 2 | 5 | 1.7301 | 0.1953 | 0.4566 |
| Spider | Neighbour | 2 | 6 | 2.6362 | 0.2148 | 0.3385 |
| Spider | Neighbour | 2 | 7 | 2.1225 | 0.2344 | 0.4398 |
| Spider | Neighbour | 2 | 8 | 2.0171 | 0.1953 | 0.5040 |
| Spider | Neighbour | 2 | 9 | 1.6782 | 0.1562 | 0.4397 |
| Spider | Neighbour | 2 | 10 | 2.2351 | 0.1953 | 0.4872 |
| Spider | Neighbour | 2 | 11 | 1.6376 | 0.1172 | 0.3360 |
| Spider | Neighbour | 2 | 12 | 1.8982 | 0.2148 | 0.3795 |
| Spider | Neighbour | 2 | 13 | 2.3052 | 0.1562 | 0.5067 |
| Spider | Neighbour | 2 | 14 | 1.7704 | 0.1758 | 0.4585 |
| Spider | Neighbour | 2 | 15 | 2.2042 | 0.2148 | 0.4651 |
| Spider | Neighbour | 2 | 16 | 2.0038 | 0.1953 | 0.4827 |
| Spider | Neighbour | 2 | 17 | 1.8974 | 0.2148 | 0.4726 |
| Spider | Neighbour | 2 | 18 | 2.2005 | 0.1660 | 0.5099 |
| Spider | Neighbour | 2 | 19 | 1.6291 | 0.1953 | 0.5339 |
| Spider | Neighbour | 2 | 20 | 2.2409 | 0.2344 | 0.5241 |
| Spider | Neighbour | 2 | 21 | 2.3960 | 0.2344 | 0.3355 |
| Spider | Neighbour | 2 | 22 | 2.4462 | 0.2344 | 0.4424 |
| Spider | Neighbour | 2 | 23 | 2.3533 | 0.2344 | 0.3326 |
| Spider | Neighbour | 2 | 24 | 1.9741 | 0.2148 | 0.3368 |
| Spider | Neighbour | 2 | 25 | 2.3006 | 0.1758 | 0.2874 |
| Spider | Neighbour | 2 | 26 | 2.1642 | 0.1758 | 0.3816 |
| Spider | Neighbour | 2 | 27 | 2.2258 | 0.1855 | 0.4546 |
| Spider | Neighbour | 2 | 28 | 2.2903 | 0.2148 | 0.3467 |
| Spider | Neighbour | 2 | 29 | 1.9478 | 0.1953 | 0.4966 |
| Spider | Neighbour | 2 | 30 | 1.8705 | 0.1953 | 0.4401 |
| Spider | Neighbour | 3 | 1 | 1.9660 | 0.2344 | 0.4662 |
| Spider | Neighbour | 3 | 2 | 1.9681 | 0.1953 | 0.4865 |
| Spider | Neighbour | 3 | 3 | 2.1216 | 0.1953 | 0.4045 |
| Spider | Neighbour | 3 | 4 | 2.3391 | 0.2148 | 0.3502 |
| Spider | Neighbour | 3 | 5 | 1.4833 | 0.1953 | 0.5150 |
| Spider | Neighbour | 3 | 6 | 2.0412 | 0.1953 | 0.4829 |
| Spider | Neighbour | 3 | 7 | 2.0244 | 0.2344 | 0.5660 |
| Spider | Neighbour | 3 | 8 | 1.8392 | 0.1953 | 0.4803 |
| Spider | Neighbour | 3 | 9 | 1.5877 | 0.2148 | 0.6297 |
| Spider | Neighbour | 3 | 10 | 1.5783 | 0.1367 | 0.5277 |
| Spider | Neighbour | 3 | 11 | 1.6680 | 0.2344 | 0.4539 |
| Spider | Neighbour | 3 | 12 | 1.9698 | 0.1562 | 0.4582 |
| Spider | Neighbour | 3 | 13 | 1.8740 | 0.1953 | 0.3842 |
| Spider | Neighbour | 3 | 14 | 2.0462 | 0.1953 | 0.4788 |
| Spider | Neighbour | 3 | 15 | 1.7816 | 0.1855 | 0.4806 |
| Spider | Neighbour | 3 | 16 | 2.3994 | 0.2148 | 0.3875 |
| Spider | Neighbour | 3 | 17 | 1.6898 | 0.1953 | 0.4973 |
| Spider | Neighbour | 3 | 18 | 2.2324 | 0.2344 | 0.5008 |
| Spider | Neighbour | 3 | 19 | 1.3734 | 0.1758 | 0.5198 |
| Spider | Neighbour | 3 | 20 | 2.1532 | 0.2344 | 0.3610 |
| Spider | Neighbour | 3 | 21 | 1.7933 | 0.1758 | 0.4409 |
| Spider | Neighbour | 3 | 22 | 2.0938 | 0.1953 | 0.4983 |
| Spider | Neighbour | 3 | 23 | 1.5340 | 0.1758 | 0.4687 |
| Spider | Neighbour | 3 | 24 | 2.0068 | 0.1562 | 0.3872 |
| Spider | Neighbour | 3 | 25 | 2.0007 | 0.1758 | 0.3865 |
| Spider | Neighbour | 3 | 26 | 2.3184 | 0.2344 | 0.3321 |
| Spider | Neighbour | 3 | 27 | 1.9956 | 0.1562 | 0.3944 |
| Spider | Neighbour | 3 | 28 | 2.0603 | 0.1855 | 0.3497 |
| Spider | Neighbour | 3 | 29 | 1.7801 | 0.1758 | 0.3519 |
| Spider | Neighbour | 3 | 30 | 1.8911 | 0.1953 | 0.4651 |
| Spider | Structured | 0 | 1 | 3.0108 | 0.2930 | 0.5009 |
| Spider | Structured | 0 | 2 | 3.1089 | 0.2441 | 0.3883 |
| Spider | Structured | 0 | 3 | 3.2929 | 0.3906 | 0.5216 |
| Spider | Structured | 0 | 4 | 2.6916 | 0.2344 | 0.5243 |
| Spider | Structured | 0 | 5 | 2.9215 | 0.2246 | 0.4813 |
| Spider | Structured | 0 | 6 | 2.8557 | 0.3027 | 0.3069 |
| Spider | Structured | 0 | 7 | 2.9539 | 0.2539 | 0.6471 |
| Spider | Structured | 0 | 8 | 3.0400 | 0.3125 | 0.5104 |
| Spider | Structured | 0 | 9 | 3.0142 | 0.2734 | 0.5605 |
| Spider | Structured | 0 | 10 | 3.3859 | 0.3516 | 0.5457 |
| Spider | Structured | 0 | 11 | 3.3486 | 0.3418 | 0.3187 |
| Spider | Structured | 0 | 12 | 2.5944 | 0.2441 | 0.4425 |
| Spider | Structured | 0 | 13 | 2.6694 | 0.1953 | 0.5236 |
| Spider | Structured | 0 | 14 | 3.1808 | 0.3027 | 0.6015 |
| Spider | Structured | 0 | 15 | 3.2614 | 0.2832 | 0.5497 |
| Spider | Structured | 0 | 16 | 3.0478 | 0.3320 | 0.4798 |
| Spider | Structured | 0 | 17 | 2.9913 | 0.2637 | 0.6001 |
| Spider | Structured | 0 | 18 | 3.0553 | 0.3125 | 0.6149 |
| Spider | Structured | 0 | 19 | 3.1596 | 0.3516 | 0.5460 |
| Spider | Structured | 0 | 20 | 2.8050 | 0.2832 | 0.6167 |
| Spider | Structured | 0 | 21 | 2.8737 | 0.2832 | 0.5605 |
| Spider | Structured | 0 | 22 | 2.5057 | 0.3223 | 0.4428 |
| Spider | Structured | 0 | 23 | 2.9488 | 0.3027 | 0.3522 |
| Spider | Structured | 0 | 24 | 3.0790 | 0.3125 | 0.3486 |
| Spider | Structured | 0 | 25 | 3.0619 | 0.3125 | 0.5431 |
| Spider | Structured | 0 | 26 | 3.0065 | 0.2539 | 0.4182 |
| Spider | Structured | 0 | 27 | 3.0153 | 0.2930 | 0.6436 |
| Spider | Structured | 0 | 28 | 2.4966 | 0.2539 | 0.5744 |
| Spider | Structured | 0 | 29 | 2.5745 | 0.2734 | 0.4116 |
| Spider | Structured | 0 | 30 | 3.0934 | 0.2734 | 0.4466 |
| Spider | Structured | 1 | 1 | 2.8270 | 0.2344 | 0.4938 |
| Spider | Structured | 1 | 2 | 2.2833 | 0.2344 | 0.3970 |
| Spider | Structured | 1 | 3 | 3.9477 | 0.3711 | 0.6200 |
| Spider | Structured | 1 | 4 | 2.2378 | 0.2637 | 0.5456 |
| Spider | Structured | 1 | 5 | 3.9536 | 0.3711 | 0.5113 |
| Spider | Structured | 1 | 6 | 2.9023 | 0.3711 | 0.5986 |
| Spider | Structured | 1 | 7 | 2.8069 | 0.2930 | 0.5388 |
| Spider | Structured | 1 | 8 | 2.2629 | 0.2051 | 0.5267 |
| Spider | Structured | 1 | 9 | 3.8038 | 0.3613 | 0.5998 |
| Spider | Structured | 1 | 10 | 2.8726 | 0.3320 | 0.5559 |
| Spider | Structured | 1 | 11 | 2.7367 | 0.2637 | 0.6182 |
| Spider | Structured | 1 | 12 | 2.3721 | 0.2344 | 0.3442 |
| Spider | Structured | 1 | 13 | 2.8057 | 0.2539 | 0.4393 |
| Spider | Structured | 1 | 14 | 3.6757 | 0.3613 | 0.5583 |
| Spider | Structured | 1 | 15 | 2.1451 | 0.3223 | 0.4544 |
| Spider | Structured | 1 | 16 | 2.7491 | 0.2832 | 0.5419 |
| Spider | Structured | 1 | 17 | 2.7249 | 0.3027 | 0.4398 |
| Spider | Structured | 1 | 18 | 2.8285 | 0.3125 | 0.4758 |
| Spider | Structured | 1 | 19 | 2.5095 | 0.2344 | 0.5750 |
| Spider | Structured | 1 | 20 | 3.3771 | 0.3223 | 0.6157 |
| Spider | Structured | 1 | 21 | 4.7412 | 0.3906 | 0.5530 |
| Spider | Structured | 1 | 22 | 3.6580 | 0.3906 | 0.4787 |
| Spider | Structured | 1 | 23 | 2.4973 | 0.2344 | 0.3680 |
| Spider | Structured | 1 | 24 | 3.3804 | 0.3125 | 0.4802 |
| Spider | Structured | 1 | 25 | 4.4254 | 0.3906 | 0.5671 |
| Spider | Structured | 1 | 26 | 2.3885 | 0.1953 | 0.5459 |
| Spider | Structured | 1 | 27 | 2.4011 | 0.2344 | 0.5553 |
| Spider | Structured | 1 | 28 | 2.5636 | 0.2930 | 0.5412 |
| Spider | Structured | 1 | 29 | 2.5801 | 0.2539 | 0.5592 |
| Spider | Structured | 1 | 30 | 2.5180 | 0.2344 | 0.2900 |
| Spider | Structured | 2 | 1 | 2.2007 | 0.1953 | 0.5935 |
| Spider | Structured | 2 | 2 | 4.8647 | 0.3906 | 0.5702 |
| Spider | Structured | 2 | 3 | 3.4741 | 0.3613 | 0.6310 |
| Spider | Structured | 2 | 4 | 4.6585 | 0.4688 | 0.5358 |
| Spider | Structured | 2 | 5 | 2.6113 | 0.2734 | 0.4660 |
| Spider | Structured | 2 | 6 | 4.8255 | 0.3906 | 0.5755 |
| Spider | Structured | 2 | 7 | 2.8613 | 0.2734 | 0.3774 |
| Spider | Structured | 2 | 8 | 2.6183 | 0.2344 | 0.5778 |
| Spider | Structured | 2 | 9 | 3.2778 | 0.3125 | 0.6154 |
| Spider | Structured | 2 | 10 | 3.1593 | 0.3613 | 0.5757 |
| Spider | Structured | 2 | 11 | 3.3815 | 0.3125 | 0.5123 |
| Spider | Structured | 2 | 12 | 2.2982 | 0.2441 | 0.5942 |
| Spider | Structured | 2 | 13 | 2.5972 | 0.2832 | 0.5647 |
| Spider | Structured | 2 | 14 | 3.5815 | 0.3223 | 0.5948 |
| Spider | Structured | 2 | 15 | 1.9781 | 0.2344 | 0.4659 |
| Spider | Structured | 2 | 16 | 2.1357 | 0.3613 | 0.5867 |
| Spider | Structured | 2 | 17 | 2.3259 | 0.2344 | 0.4926 |
| Spider | Structured | 2 | 18 | 2.4457 | 0.3027 | 0.5663 |
| Spider | Structured | 2 | 19 | 2.9120 | 0.3223 | 0.4388 |
| Spider | Structured | 2 | 20 | 3.3006 | 0.3906 | 0.5392 |
| Spider | Structured | 2 | 21 | 3.1655 | 0.3613 | 0.5748 |
| Spider | Structured | 2 | 22 | 3.1432 | 0.3516 | 0.4899 |
| Spider | Structured | 2 | 23 | 2.2301 | 0.2246 | 0.5389 |
| Spider | Structured | 2 | 24 | 4.0501 | 0.4199 | 0.6158 |
| Spider | Structured | 2 | 25 | 3.4789 | 0.3516 | 0.5146 |
| Spider | Structured | 2 | 26 | 2.0065 | 0.2734 | 0.4936 |
| Spider | Structured | 2 | 27 | 3.6248 | 0.2832 | 0.6289 |
| Spider | Structured | 2 | 28 | 2.2052 | 0.2148 | 0.4640 |
| Spider | Structured | 2 | 29 | 2.2306 | 0.2344 | 0.4391 |
| Spider | Structured | 2 | 30 | 2.2381 | 0.2344 | 0.4623 |
| Spider | Structured | 3 | 1 | 3.4745 | 0.3516 | 0.5995 |
| Spider | Structured | 3 | 2 | 2.5477 | 0.2832 | 0.6183 |
| Spider | Structured | 3 | 3 | 2.1933 | 0.2344 | 0.3834 |
| Spider | Structured | 3 | 4 | 3.6246 | 0.3906 | 0.5632 |
| Spider | Structured | 3 | 5 | 2.7235 | 0.2344 | 0.5143 |
| Spider | Structured | 3 | 6 | 2.1257 | 0.2637 | 0.5435 |
| Spider | Structured | 3 | 7 | 2.0261 | 0.1953 | 0.5687 |
| Spider | Structured | 3 | 8 | 2.5016 | 0.3125 | 0.4831 |
| Spider | Structured | 3 | 9 | 3.6728 | 0.3906 | 0.5782 |
| Spider | Structured | 3 | 10 | 1.8143 | 0.2246 | 0.4814 |
| Spider | Structured | 3 | 11 | 1.7610 | 0.2832 | 0.4331 |
| Spider | Structured | 3 | 12 | 4.2103 | 0.3906 | 0.5596 |
| Spider | Structured | 3 | 13 | 2.2012 | 0.2344 | 0.5018 |
| Spider | Structured | 3 | 14 | 2.6202 | 0.2734 | 0.6345 |
| Spider | Structured | 3 | 15 | 3.2038 | 0.3906 | 0.5276 |
| Spider | Structured | 3 | 16 | 4.0569 | 0.3906 | 0.5810 |
| Spider | Structured | 3 | 17 | 3.7006 | 0.3906 | 0.5403 |
| Spider | Structured | 3 | 18 | 4.1060 | 0.3906 | 0.6295 |
| Spider | Structured | 3 | 19 | 3.2836 | 0.3125 | 0.6107 |
| Spider | Structured | 3 | 20 | 2.9316 | 0.3516 | 0.5479 |
| Spider | Structured | 3 | 21 | 2.7573 | 0.3613 | 0.5438 |
| Spider | Structured | 3 | 22 | 3.8131 | 0.3906 | 0.5349 |
| Spider | Structured | 3 | 23 | 2.4768 | 0.3125 | 0.5204 |
| Spider | Structured | 3 | 24 | 2.4533 | 0.3320 | 0.4223 |
| Spider | Structured | 3 | 25 | 2.0805 | 0.3223 | 0.5729 |
| Spider | Structured | 3 | 26 | 2.4142 | 0.2539 | 0.5381 |
| Spider | Structured | 3 | 27 | 4.8614 | 0.3906 | 0.5773 |
| Spider | Structured | 3 | 28 | 2.7282 | 0.2344 | 0.3989 |
| Spider | Structured | 3 | 29 | 1.9798 | 0.2441 | 0.3037 |
| Spider | Structured | 3 | 30 | 4.4371 | 0.3906 | 0.6170 |
| Gecko | No coupling | 0 | 1 | 1.2744 | 0.1432 | 0.4310 |
| Gecko | No coupling | 0 | 2 | 1.5681 | 0.1302 | 0.3863 |
| Gecko | No coupling | 0 | 3 | 2.0542 | 0.1432 | 0.4569 |
| Gecko | No coupling | 0 | 4 | 1.9313 | 0.1432 | 0.4981 |
| Gecko | No coupling | 0 | 5 | 2.2716 | 0.1562 | 0.4919 |
| Gecko | No coupling | 0 | 6 | 1.5149 | 0.1302 | 0.4915 |
| Gecko | No coupling | 0 | 7 | 1.6543 | 0.1432 | 0.4275 |
| Gecko | No coupling | 0 | 8 | 1.8595 | 0.1432 | 0.4923 |
| Gecko | No coupling | 0 | 9 | 1.7127 | 0.1432 | 0.6349 |
| Gecko | No coupling | 0 | 10 | 1.7098 | 0.1302 | 0.3960 |
| Gecko | No coupling | 0 | 11 | 1.9210 | 0.1562 | 0.4605 |
| Gecko | No coupling | 0 | 12 | 1.5100 | 0.1302 | 0.4891 |
| Gecko | No coupling | 0 | 13 | 1.5561 | 0.1432 | 0.3943 |
| Gecko | No coupling | 0 | 14 | 2.4104 | 0.1562 | 0.4411 |
| Gecko | No coupling | 0 | 15 | 2.1314 | 0.1562 | 0.4572 |
| Gecko | No coupling | 0 | 16 | 1.3881 | 0.1432 | 0.2794 |
| Gecko | No coupling | 0 | 17 | 1.7033 | 0.1302 | 0.5624 |
| Gecko | No coupling | 0 | 18 | 2.1937 | 0.1432 | 0.4979 |
| Gecko | No coupling | 0 | 19 | 2.2126 | 0.1562 | 0.5021 |
| Gecko | No coupling | 0 | 20 | 1.7641 | 0.1432 | 0.4367 |
| Gecko | No coupling | 0 | 21 | 1.7936 | 0.1432 | 0.5668 |
| Gecko | No coupling | 0 | 22 | 1.4547 | 0.1432 | 0.3738 |
| Gecko | No coupling | 0 | 23 | 1.3934 | 0.1302 | 0.4550 |
| Gecko | No coupling | 0 | 24 | 1.7840 | 0.1432 | 0.6020 |
| Gecko | No coupling | 0 | 25 | 1.5725 | 0.1302 | 0.4916 |
| Gecko | No coupling | 0 | 26 | 1.6664 | 0.1432 | 0.5740 |
| Gecko | No coupling | 0 | 27 | 1.7116 | 0.1432 | 0.4453 |
| Gecko | No coupling | 0 | 28 | 1.6627 | 0.1302 | 0.3930 |
| Gecko | No coupling | 0 | 29 | 1.3673 | 0.1432 | 0.3290 |
| Gecko | No coupling | 0 | 30 | 1.9184 | 0.1432 | 0.6010 |
| Gecko | No coupling | 1 | 1 | 1.6304 | 0.1302 | 0.5097 |
| Gecko | No coupling | 1 | 2 | 2.0714 | 0.1432 | 0.5198 |
| Gecko | No coupling | 1 | 3 | 1.5552 | 0.1302 | 0.5002 |
| Gecko | No coupling | 1 | 4 | 1.5849 | 0.1432 | 0.6237 |
| Gecko | No coupling | 1 | 5 | 1.6851 | 0.1302 | 0.4227 |
| Gecko | No coupling | 1 | 6 | 1.9441 | 0.1562 | 0.3716 |
| Gecko | No coupling | 1 | 7 | 1.9569 | 0.1562 | 0.3633 |
| Gecko | No coupling | 1 | 8 | 1.3249 | 0.1302 | 0.4503 |
| Gecko | No coupling | 1 | 9 | 2.0620 | 0.1562 | 0.4357 |
| Gecko | No coupling | 1 | 10 | 1.5989 | 0.1432 | 0.6032 |
| Gecko | No coupling | 1 | 11 | 1.4388 | 0.1302 | 0.4685 |
| Gecko | No coupling | 1 | 12 | 1.6420 | 0.1302 | 0.5928 |
| Gecko | No coupling | 1 | 13 | 1.4450 | 0.1302 | 0.4687 |
| Gecko | No coupling | 1 | 14 | 1.4932 | 0.1302 | 0.4096 |
| Gecko | No coupling | 1 | 15 | 2.3133 | 0.1562 | 0.4720 |
| Gecko | No coupling | 1 | 16 | 1.4477 | 0.1432 | 0.5671 |
| Gecko | No coupling | 1 | 17 | 1.7990 | 0.1302 | 0.4417 |
| Gecko | No coupling | 1 | 18 | 1.8172 | 0.1432 | 0.4754 |
| Gecko | No coupling | 1 | 19 | 1.7715 | 0.1432 | 0.4227 |
| Gecko | No coupling | 1 | 20 | 1.3529 | 0.1432 | 0.6351 |
| Gecko | No coupling | 1 | 21 | 1.3537 | 0.1302 | 0.4563 |
| Gecko | No coupling | 1 | 22 | 1.3356 | 0.1562 | 0.3216 |
| Gecko | No coupling | 1 | 23 | 2.0252 | 0.1562 | 0.5505 |
| Gecko | No coupling | 1 | 24 | 2.0373 | 0.1562 | 0.4398 |
| Gecko | No coupling | 1 | 25 | 2.0147 | 0.1562 | 0.4891 |
| Gecko | No coupling | 1 | 26 | 2.0625 | 0.1562 | 0.5341 |
| Gecko | No coupling | 1 | 27 | 1.3225 | 0.1302 | 0.4782 |
| Gecko | No coupling | 1 | 28 | 1.8682 | 0.1432 | 0.4341 |
| Gecko | No coupling | 1 | 29 | 1.5925 | 0.1172 | 0.5676 |
| Gecko | No coupling | 1 | 30 | 1.8148 | 0.1562 | 0.4588 |
| Gecko | No coupling | 2 | 1 | 1.3573 | 0.1172 | 0.4747 |
| Gecko | No coupling | 2 | 2 | 1.5042 | 0.1432 | 0.4926 |
| Gecko | No coupling | 2 | 3 | 1.8770 | 0.1432 | 0.4447 |
| Gecko | No coupling | 2 | 4 | 1.7647 | 0.1302 | 0.5600 |
| Gecko | No coupling | 2 | 5 | 0.7988 | 0.1042 | 0.5003 |
| Gecko | No coupling | 2 | 6 | 1.2383 | 0.1172 | 0.4784 |
| Gecko | No coupling | 2 | 7 | 0.9903 | 0.1172 | 0.4367 |
| Gecko | No coupling | 2 | 8 | 1.4243 | 0.1172 | 0.4928 |
| Gecko | No coupling | 2 | 9 | 1.1173 | 0.1302 | 0.5232 |
| Gecko | No coupling | 2 | 10 | 1.7106 | 0.1302 | 0.4726 |
| Gecko | No coupling | 2 | 11 | 1.8668 | 0.1432 | 0.4585 |
| Gecko | No coupling | 2 | 12 | 1.0086 | 0.1042 | 0.5621 |
| Gecko | No coupling | 2 | 13 | 1.6437 | 0.1302 | 0.4334 |
| Gecko | No coupling | 2 | 14 | 1.3912 | 0.1302 | 0.4742 |
| Gecko | No coupling | 2 | 15 | 1.6391 | 0.1302 | 0.5235 |
| Gecko | No coupling | 2 | 16 | 1.6991 | 0.1432 | 0.4943 |
| Gecko | No coupling | 2 | 17 | 1.3225 | 0.1302 | 0.4976 |
| Gecko | No coupling | 2 | 18 | 1.9063 | 0.1302 | 0.4602 |
| Gecko | No coupling | 2 | 19 | 1.4815 | 0.1562 | 0.5523 |
| Gecko | No coupling | 2 | 20 | 1.8220 | 0.1562 | 0.5862 |
| Gecko | No coupling | 2 | 21 | 1.3023 | 0.1172 | 0.4493 |
| Gecko | No coupling | 2 | 22 | 1.8891 | 0.1302 | 0.4590 |
| Gecko | No coupling | 2 | 23 | 1.3956 | 0.1172 | 0.5803 |
| Gecko | No coupling | 2 | 24 | 1.3635 | 0.1302 | 0.5056 |
| Gecko | No coupling | 2 | 25 | 1.0328 | 0.1172 | 0.5210 |
| Gecko | No coupling | 2 | 26 | 1.0181 | 0.1172 | 0.5002 |
| Gecko | No coupling | 2 | 27 | 1.8930 | 0.1302 | 0.4616 |
| Gecko | No coupling | 2 | 28 | 1.3452 | 0.1432 | 0.4009 |
| Gecko | No coupling | 2 | 29 | 1.9520 | 0.1432 | 0.4152 |
| Gecko | No coupling | 2 | 30 | 1.3162 | 0.1432 | 0.3545 |
| Gecko | No coupling | 3 | 1 | 1.3331 | 0.1432 | 0.4553 |
| Gecko | No coupling | 3 | 2 | 1.4876 | 0.1432 | 0.6185 |
| Gecko | No coupling | 3 | 3 | 1.6480 | 0.1172 | 0.5290 |
| Gecko | No coupling | 3 | 4 | 0.9119 | 0.1042 | 0.5253 |
| Gecko | No coupling | 3 | 5 | 1.8827 | 0.1432 | 0.4127 |
| Gecko | No coupling | 3 | 6 | 1.2333 | 0.1172 | 0.4772 |
| Gecko | No coupling | 3 | 7 | 0.6956 | 0.0911 | 0.5632 |
| Gecko | No coupling | 3 | 8 | 1.3406 | 0.1302 | 0.4634 |
| Gecko | No coupling | 3 | 9 | 1.3602 | 0.1172 | 0.4870 |
| Gecko | No coupling | 3 | 10 | 1.1576 | 0.1302 | 0.5506 |
| Gecko | No coupling | 3 | 11 | 1.3079 | 0.1432 | 0.5220 |
| Gecko | No coupling | 3 | 12 | 1.8975 | 0.1302 | 0.4602 |
| Gecko | No coupling | 3 | 13 | 0.6167 | 0.0781 | 0.4195 |
| Gecko | No coupling | 3 | 14 | 1.8632 | 0.1302 | 0.4647 |
| Gecko | No coupling | 3 | 15 | 1.8680 | 0.1432 | 0.4486 |
| Gecko | No coupling | 3 | 16 | 1.1995 | 0.1302 | 0.4804 |
| Gecko | No coupling | 3 | 17 | 1.0493 | 0.1172 | 0.5724 |
| Gecko | No coupling | 3 | 18 | 1.6683 | 0.1302 | 0.5654 |
| Gecko | No coupling | 3 | 19 | 1.2673 | 0.1302 | 0.3924 |
| Gecko | No coupling | 3 | 20 | 1.6138 | 0.1302 | 0.5435 |
| Gecko | No coupling | 3 | 21 | 1.2149 | 0.1172 | 0.4695 |
| Gecko | No coupling | 3 | 22 | 0.4852 | 0.0781 | 0.4463 |
| Gecko | No coupling | 3 | 23 | 1.2783 | 0.1302 | 0.4871 |
| Gecko | No coupling | 3 | 24 | 1.1688 | 0.0781 | 0.4739 |
| Gecko | No coupling | 3 | 25 | 1.4234 | 0.1302 | 0.5196 |
| Gecko | No coupling | 3 | 26 | 1.8996 | 0.1302 | 0.4596 |
| Gecko | No coupling | 3 | 27 | 1.1378 | 0.1302 | 0.4946 |
| Gecko | No coupling | 3 | 28 | 0.9176 | 0.1302 | 0.6023 |
| Gecko | No coupling | 3 | 29 | 1.6732 | 0.1302 | 0.4859 |
| Gecko | No coupling | 3 | 30 | 0.3655 | 0.0781 | 0.4663 |
| Gecko | Neighbour | 0 | 1 | 1.5499 | 0.1823 | 0.3804 |
| Gecko | Neighbour | 0 | 2 | 1.5733 | 0.2214 | 0.2970 |
| Gecko | Neighbour | 0 | 3 | 2.3404 | 0.2474 | 0.4774 |
| Gecko | Neighbour | 0 | 4 | 1.5052 | 0.1823 | 0.4199 |
| Gecko | Neighbour | 0 | 5 | 1.4266 | 0.1823 | 0.3864 |
| Gecko | Neighbour | 0 | 6 | 1.8084 | 0.2214 | 0.3360 |
| Gecko | Neighbour | 0 | 7 | 1.5911 | 0.1953 | 0.3916 |
| Gecko | Neighbour | 0 | 8 | 1.7494 | 0.2214 | 0.4734 |
| Gecko | Neighbour | 0 | 9 | 1.7682 | 0.2474 | 0.3866 |
| Gecko | Neighbour | 0 | 10 | 1.4345 | 0.2344 | 0.5279 |
| Gecko | Neighbour | 0 | 11 | 1.5239 | 0.2214 | 0.4336 |
| Gecko | Neighbour | 0 | 12 | 1.3435 | 0.2083 | 0.3574 |
| Gecko | Neighbour | 0 | 13 | 1.8878 | 0.2474 | 0.4124 |
| Gecko | Neighbour | 0 | 14 | 1.6817 | 0.2214 | 0.4672 |
| Gecko | Neighbour | 0 | 15 | 1.4280 | 0.2214 | 0.4193 |
| Gecko | Neighbour | 0 | 16 | 1.6485 | 0.2604 | 0.4176 |
| Gecko | Neighbour | 0 | 17 | 1.5936 | 0.2474 | 0.3881 |
| Gecko | Neighbour | 0 | 18 | 1.5144 | 0.2083 | 0.3303 |
| Gecko | Neighbour | 0 | 19 | 1.3433 | 0.2604 | 0.4141 |
| Gecko | Neighbour | 0 | 20 | 1.4814 | 0.2344 | 0.3937 |
| Gecko | Neighbour | 0 | 21 | 1.3075 | 0.2474 | 0.3131 |
| Gecko | Neighbour | 0 | 22 | 1.4613 | 0.2083 | 0.4707 |
| Gecko | Neighbour | 0 | 23 | 1.3144 | 0.2604 | 0.4070 |
| Gecko | Neighbour | 0 | 24 | 2.3793 | 0.2474 | 0.4770 |
| Gecko | Neighbour | 0 | 25 | 1.8113 | 0.2214 | 0.4565 |
| Gecko | Neighbour | 0 | 26 | 1.4270 | 0.2083 | 0.4994 |
| Gecko | Neighbour | 0 | 27 | 1.7956 | 0.2214 | 0.3148 |
| Gecko | Neighbour | 0 | 28 | 1.3578 | 0.2474 | 0.3420 |
| Gecko | Neighbour | 0 | 29 | 1.4123 | 0.2474 | 0.3960 |
| Gecko | Neighbour | 0 | 30 | 1.5871 | 0.2083 | 0.3985 |
| Gecko | Neighbour | 1 | 1 | 2.0353 | 0.2474 | 0.4691 |
| Gecko | Neighbour | 1 | 2 | 1.4984 | 0.2344 | 0.4770 |
| Gecko | Neighbour | 1 | 3 | 2.0040 | 0.2474 | 0.4735 |
| Gecko | Neighbour | 1 | 4 | 1.8628 | 0.2344 | 0.4508 |
| Gecko | Neighbour | 1 | 5 | 2.3786 | 0.2344 | 0.4772 |
| Gecko | Neighbour | 1 | 6 | 1.8639 | 0.1953 | 0.4428 |
| Gecko | Neighbour | 1 | 7 | 1.4474 | 0.1823 | 0.3315 |
| Gecko | Neighbour | 1 | 8 | 2.1682 | 0.2474 | 0.4704 |
| Gecko | Neighbour | 1 | 9 | 1.6062 | 0.2214 | 0.4749 |
| Gecko | Neighbour | 1 | 10 | 1.9284 | 0.2474 | 0.4542 |
| Gecko | Neighbour | 1 | 11 | 1.3000 | 0.2214 | 0.4650 |
| Gecko | Neighbour | 1 | 12 | 2.1122 | 0.2474 | 0.4651 |
| Gecko | Neighbour | 1 | 13 | 2.3572 | 0.2344 | 0.4917 |
| Gecko | Neighbour | 1 | 14 | 1.6370 | 0.1823 | 0.3864 |
| Gecko | Neighbour | 1 | 15 | 1.0739 | 0.2344 | 0.4423 |
| Gecko | Neighbour | 1 | 16 | 2.1046 | 0.2474 | 0.4736 |
| Gecko | Neighbour | 1 | 17 | 2.3789 | 0.2344 | 0.4773 |
| Gecko | Neighbour | 1 | 18 | 1.9527 | 0.2474 | 0.4586 |
| Gecko | Neighbour | 1 | 19 | 2.2163 | 0.2474 | 0.4813 |
| Gecko | Neighbour | 1 | 20 | 1.5566 | 0.1953 | 0.3204 |
| Gecko | Neighbour | 1 | 21 | 1.5182 | 0.1953 | 0.4298 |
| Gecko | Neighbour | 1 | 22 | 2.2138 | 0.2474 | 0.4804 |
| Gecko | Neighbour | 1 | 23 | 1.6519 | 0.2214 | 0.4510 |
| Gecko | Neighbour | 1 | 24 | 1.7476 | 0.2083 | 0.4743 |
| Gecko | Neighbour | 1 | 25 | 1.6996 | 0.2474 | 0.4566 |
| Gecko | Neighbour | 1 | 26 | 1.6144 | 0.2604 | 0.3501 |
| Gecko | Neighbour | 1 | 27 | 1.5123 | 0.1823 | 0.4595 |
| Gecko | Neighbour | 1 | 28 | 2.3897 | 0.2344 | 0.4681 |
| Gecko | Neighbour | 1 | 29 | 1.5646 | 0.2474 | 0.4826 |
| Gecko | Neighbour | 1 | 30 | 1.8590 | 0.2344 | 0.4475 |
| Gecko | Neighbour | 2 | 1 | 2.3555 | 0.2344 | 0.4899 |
| Gecko | Neighbour | 2 | 2 | 1.9322 | 0.2344 | 0.5103 |
| Gecko | Neighbour | 2 | 3 | 2.3438 | 0.1823 | 0.4730 |
| Gecko | Neighbour | 2 | 4 | 1.6930 | 0.2344 | 0.4773 |
| Gecko | Neighbour | 2 | 5 | 2.3551 | 0.2344 | 0.4872 |
| Gecko | Neighbour | 2 | 6 | 1.3896 | 0.2214 | 0.4099 |
| Gecko | Neighbour | 2 | 7 | 1.7883 | 0.2083 | 0.4717 |
| Gecko | Neighbour | 2 | 8 | 2.3793 | 0.2344 | 0.4745 |
| Gecko | Neighbour | 2 | 9 | 1.5189 | 0.2214 | 0.4351 |
| Gecko | Neighbour | 2 | 10 | 1.5324 | 0.2214 | 0.4990 |
| Gecko | Neighbour | 2 | 11 | 1.4614 | 0.2344 | 0.4462 |
| Gecko | Neighbour | 2 | 12 | 1.8946 | 0.2344 | 0.4857 |
| Gecko | Neighbour | 2 | 13 | 2.3795 | 0.2344 | 0.4747 |
| Gecko | Neighbour | 2 | 14 | 1.6886 | 0.2083 | 0.4906 |
| Gecko | Neighbour | 2 | 15 | 2.0752 | 0.2344 | 0.5595 |
| Gecko | Neighbour | 2 | 16 | 1.4069 | 0.2474 | 0.4779 |
| Gecko | Neighbour | 2 | 17 | 1.8798 | 0.1823 | 0.5124 |
| Gecko | Neighbour | 2 | 18 | 2.3545 | 0.2344 | 0.4891 |
| Gecko | Neighbour | 2 | 19 | 2.1909 | 0.2344 | 0.5151 |
| Gecko | Neighbour | 2 | 20 | 1.6454 | 0.1823 | 0.4470 |
| Gecko | Neighbour | 2 | 21 | 1.8692 | 0.2344 | 0.4605 |
| Gecko | Neighbour | 2 | 22 | 2.3800 | 0.2344 | 0.4746 |
| Gecko | Neighbour | 2 | 23 | 2.1930 | 0.2344 | 0.5263 |
| Gecko | Neighbour | 2 | 24 | 2.3790 | 0.2344 | 0.4783 |
| Gecko | Neighbour | 2 | 25 | 2.1986 | 0.2344 | 0.5066 |
| Gecko | Neighbour | 2 | 26 | 1.5877 | 0.2344 | 0.4156 |
| Gecko | Neighbour | 2 | 27 | 1.5000 | 0.2083 | 0.5080 |
| Gecko | Neighbour | 2 | 28 | 1.8195 | 0.2344 | 0.5106 |
| Gecko | Neighbour | 2 | 29 | 1.5723 | 0.1953 | 0.4398 |
| Gecko | Neighbour | 2 | 30 | 2.0596 | 0.2344 | 0.5282 |
| Gecko | Neighbour | 3 | 1 | 1.4236 | 0.2344 | 0.4732 |
| Gecko | Neighbour | 3 | 2 | 2.3760 | 0.2344 | 0.4753 |
| Gecko | Neighbour | 3 | 3 | 2.3797 | 0.2344 | 0.4749 |
| Gecko | Neighbour | 3 | 4 | 1.5059 | 0.1693 | 0.4413 |
| Gecko | Neighbour | 3 | 5 | 1.6867 | 0.2083 | 0.4909 |
| Gecko | Neighbour | 3 | 6 | 1.5626 | 0.2344 | 0.4597 |
| Gecko | Neighbour | 3 | 7 | 1.7597 | 0.2344 | 0.4906 |
| Gecko | Neighbour | 3 | 8 | 2.4274 | 0.2344 | 0.5238 |
| Gecko | Neighbour | 3 | 9 | 2.1912 | 0.2344 | 0.5139 |
| Gecko | Neighbour | 3 | 10 | 1.9606 | 0.2344 | 0.5346 |
| Gecko | Neighbour | 3 | 11 | 0.9110 | 0.1953 | 0.5391 |
| Gecko | Neighbour | 3 | 12 | 1.8162 | 0.2344 | 0.4753 |
| Gecko | Neighbour | 3 | 13 | 2.3792 | 0.2344 | 0.4773 |
| Gecko | Neighbour | 3 | 14 | 1.2494 | 0.2214 | 0.4863 |
| Gecko | Neighbour | 3 | 15 | 1.5858 | 0.1823 | 0.4884 |
| Gecko | Neighbour | 3 | 16 | 1.5610 | 0.2344 | 0.4587 |
| Gecko | Neighbour | 3 | 17 | 2.3782 | 0.2344 | 0.4777 |
| Gecko | Neighbour | 3 | 18 | 1.6125 | 0.2344 | 0.5203 |
| Gecko | Neighbour | 3 | 19 | 2.3801 | 0.2344 | 0.4747 |
| Gecko | Neighbour | 3 | 20 | 1.6483 | 0.2344 | 0.4273 |
| Gecko | Neighbour | 3 | 21 | 1.4398 | 0.2214 | 0.4042 |
| Gecko | Neighbour | 3 | 22 | 2.3560 | 0.2344 | 0.4878 |
| Gecko | Neighbour | 3 | 23 | 1.6432 | 0.2083 | 0.4212 |
| Gecko | Neighbour | 3 | 24 | 1.5776 | 0.2344 | 0.4611 |
| Gecko | Neighbour | 3 | 25 | 1.7915 | 0.1823 | 0.5056 |
| Gecko | Neighbour | 3 | 26 | 1.9625 | 0.2344 | 0.5361 |
| Gecko | Neighbour | 3 | 27 | 1.5290 | 0.2344 | 0.4623 |
| Gecko | Neighbour | 3 | 28 | 2.1928 | 0.2344 | 0.5142 |
| Gecko | Neighbour | 3 | 29 | 1.3307 | 0.2344 | 0.4469 |
| Gecko | Neighbour | 3 | 30 | 2.3794 | 0.2344 | 0.4748 |
| Gecko | Structured | 0 | 1 | 4.5676 | 0.3906 | 0.6747 |
| Gecko | Structured | 0 | 2 | 4.7269 | 0.4297 | 0.6834 |
| Gecko | Structured | 0 | 3 | 5.1999 | 0.3906 | 0.6869 |
| Gecko | Structured | 0 | 4 | 4.7024 | 0.3906 | 0.6747 |
| Gecko | Structured | 0 | 5 | 4.1979 | 0.3906 | 0.6801 |
| Gecko | Structured | 0 | 6 | 4.4860 | 0.3906 | 0.6803 |
| Gecko | Structured | 0 | 7 | 5.1435 | 0.3906 | 0.6889 |
| Gecko | Structured | 0 | 8 | 4.7225 | 0.4688 | 0.6844 |
| Gecko | Structured | 0 | 9 | 3.6193 | 0.3906 | 0.4591 |
| Gecko | Structured | 0 | 10 | 4.7179 | 0.4688 | 0.5526 |
| Gecko | Structured | 0 | 11 | 5.0886 | 0.3906 | 0.6989 |
| Gecko | Structured | 0 | 12 | 4.6927 | 0.4688 | 0.5961 |
| Gecko | Structured | 0 | 13 | 4.6976 | 0.3906 | 0.6764 |
| Gecko | Structured | 0 | 14 | 4.0016 | 0.3125 | 0.4831 |
| Gecko | Structured | 0 | 15 | 4.7304 | 0.3906 | 0.6794 |
| Gecko | Structured | 0 | 16 | 4.4955 | 0.3906 | 0.6533 |
| Gecko | Structured | 0 | 17 | 4.7263 | 0.4688 | 0.5603 |
| Gecko | Structured | 0 | 18 | 4.9617 | 0.3906 | 0.6650 |
| Gecko | Structured | 0 | 19 | 3.6521 | 0.3125 | 0.3890 |
| Gecko | Structured | 0 | 20 | 4.7311 | 0.4297 | 0.6832 |
| Gecko | Structured | 0 | 21 | 4.8058 | 0.4688 | 0.4839 |
| Gecko | Structured | 0 | 22 | 3.3535 | 0.3906 | 0.5408 |
| Gecko | Structured | 0 | 23 | 5.0271 | 0.3906 | 0.7002 |
| Gecko | Structured | 0 | 24 | 3.3846 | 0.3906 | 0.6337 |
| Gecko | Structured | 0 | 25 | 4.7877 | 0.3906 | 0.6795 |
| Gecko | Structured | 0 | 26 | 4.7273 | 0.4688 | 0.6168 |
| Gecko | Structured | 0 | 27 | 4.5414 | 0.4557 | 0.4815 |
| Gecko | Structured | 0 | 28 | 4.6413 | 0.3906 | 0.6733 |
| Gecko | Structured | 0 | 29 | 3.2013 | 0.3125 | 0.3263 |
| Gecko | Structured | 0 | 30 | 3.7693 | 0.3125 | 0.5493 |
| Gecko | Structured | 1 | 1 | 4.6365 | 0.3906 | 0.6763 |
| Gecko | Structured | 1 | 2 | 3.7024 | 0.3906 | 0.5973 |
| Gecko | Structured | 1 | 3 | 5.2694 | 0.3906 | 0.6713 |
| Gecko | Structured | 1 | 4 | 3.4629 | 0.3125 | 0.4874 |
| Gecko | Structured | 1 | 5 | 4.6464 | 0.3906 | 0.6816 |
| Gecko | Structured | 1 | 6 | 3.8536 | 0.3906 | 0.4264 |
| Gecko | Structured | 1 | 7 | 5.0092 | 0.3906 | 0.6942 |
| Gecko | Structured | 1 | 8 | 4.6485 | 0.3906 | 0.6540 |
| Gecko | Structured | 1 | 9 | 4.8051 | 0.3906 | 0.7102 |
| Gecko | Structured | 1 | 10 | 4.7934 | 0.3906 | 0.6757 |
| Gecko | Structured | 1 | 11 | 4.6334 | 0.3906 | 0.6760 |
| Gecko | Structured | 1 | 12 | 5.0387 | 0.3906 | 0.6940 |
| Gecko | Structured | 1 | 13 | 3.3085 | 0.3906 | 0.6185 |
| Gecko | Structured | 1 | 14 | 5.0452 | 0.3906 | 0.6940 |
| Gecko | Structured | 1 | 15 | 4.7016 | 0.3906 | 0.6693 |
| Gecko | Structured | 1 | 16 | 3.6613 | 0.3906 | 0.5394 |
| Gecko | Structured | 1 | 17 | 2.5295 | 0.3906 | 0.2748 |
| Gecko | Structured | 1 | 18 | 5.0206 | 0.3906 | 0.7014 |
| Gecko | Structured | 1 | 19 | 4.4399 | 0.3906 | 0.6923 |
| Gecko | Structured | 1 | 20 | 4.4505 | 0.4688 | 0.3964 |
| Gecko | Structured | 1 | 21 | 4.7939 | 0.4688 | 0.4911 |
| Gecko | Structured | 1 | 22 | 5.0317 | 0.3906 | 0.6922 |
| Gecko | Structured | 1 | 23 | 4.6911 | 0.4688 | 0.5586 |
| Gecko | Structured | 1 | 24 | 3.3199 | 0.3906 | 0.5762 |
| Gecko | Structured | 1 | 25 | 4.2622 | 0.3906 | 0.5033 |
| Gecko | Structured | 1 | 26 | 4.9677 | 0.3906 | 0.7034 |
| Gecko | Structured | 1 | 27 | 4.7338 | 0.3906 | 0.6827 |
| Gecko | Structured | 1 | 28 | 3.7117 | 0.3906 | 0.4721 |
| Gecko | Structured | 1 | 29 | 3.2574 | 0.3906 | 0.4444 |
| Gecko | Structured | 1 | 30 | 3.6354 | 0.3906 | 0.5086 |
| Gecko | Structured | 2 | 1 | 3.1409 | 0.3906 | 0.6006 |
| Gecko | Structured | 2 | 2 | 4.2934 | 0.4688 | 0.4229 |
| Gecko | Structured | 2 | 3 | 3.3790 | 0.3906 | 0.5070 |
| Gecko | Structured | 2 | 4 | 3.4188 | 0.3906 | 0.4537 |
| Gecko | Structured | 2 | 5 | 4.0937 | 0.3906 | 0.6269 |
| Gecko | Structured | 2 | 6 | 3.4403 | 0.3125 | 0.5073 |
| Gecko | Structured | 2 | 7 | 4.8216 | 0.3906 | 0.7090 |
| Gecko | Structured | 2 | 8 | 4.6262 | 0.3906 | 0.6840 |
| Gecko | Structured | 2 | 9 | 4.3937 | 0.3906 | 0.4249 |
| Gecko | Structured | 2 | 10 | 4.2564 | 0.4688 | 0.4067 |
| Gecko | Structured | 2 | 11 | 4.7749 | 0.3906 | 0.7109 |
| Gecko | Structured | 2 | 12 | 4.3664 | 0.4688 | 0.4111 |
| Gecko | Structured | 2 | 13 | 4.5732 | 0.3906 | 0.6782 |
| Gecko | Structured | 2 | 14 | 3.1332 | 0.3906 | 0.5540 |
| Gecko | Structured | 2 | 15 | 4.6572 | 0.3906 | 0.6683 |
| Gecko | Structured | 2 | 16 | 3.4628 | 0.3125 | 0.5357 |
| Gecko | Structured | 2 | 17 | 3.6332 | 0.3906 | 0.3837 |
| Gecko | Structured | 2 | 18 | 4.6939 | 0.3906 | 0.5793 |
| Gecko | Structured | 2 | 19 | 3.6848 | 0.3906 | 0.5524 |
| Gecko | Structured | 2 | 20 | 4.3967 | 0.4688 | 0.4077 |
| Gecko | Structured | 2 | 21 | 4.5639 | 0.3906 | 0.6836 |
| Gecko | Structured | 2 | 22 | 2.9292 | 0.3906 | 0.5591 |
| Gecko | Structured | 2 | 23 | 4.4054 | 0.3906 | 0.6848 |
| Gecko | Structured | 2 | 24 | 5.1733 | 0.3906 | 0.6813 |
| Gecko | Structured | 2 | 25 | 3.4111 | 0.3125 | 0.5034 |
| Gecko | Structured | 2 | 26 | 3.6021 | 0.3906 | 0.5776 |
| Gecko | Structured | 2 | 27 | 3.5164 | 0.3906 | 0.3854 |
| Gecko | Structured | 2 | 28 | 2.9020 | 0.3906 | 0.5084 |
| Gecko | Structured | 2 | 29 | 2.8120 | 0.3125 | 0.5746 |
| Gecko | Structured | 2 | 30 | 3.0349 | 0.3906 | 0.3596 |
| Gecko | Structured | 3 | 1 | 2.9450 | 0.3906 | 0.4187 |
| Gecko | Structured | 3 | 2 | 3.4853 | 0.3906 | 0.4088 |
| Gecko | Structured | 3 | 3 | 3.7050 | 0.3906 | 0.3874 |
| Gecko | Structured | 3 | 4 | 3.0535 | 0.3125 | 0.5783 |
| Gecko | Structured | 3 | 5 | 3.4544 | 0.3906 | 0.4713 |
| Gecko | Structured | 3 | 6 | 2.9710 | 0.3125 | 0.4056 |
| Gecko | Structured | 3 | 7 | 4.0328 | 0.3125 | 0.5353 |
| Gecko | Structured | 3 | 8 | 4.4718 | 0.4688 | 0.5329 |
| Gecko | Structured | 3 | 9 | 3.2976 | 0.3125 | 0.4204 |
| Gecko | Structured | 3 | 10 | 3.9542 | 0.3906 | 0.4969 |
| Gecko | Structured | 3 | 11 | 2.8308 | 0.3125 | 0.5139 |
| Gecko | Structured | 3 | 12 | 4.0539 | 0.3125 | 0.5301 |
| Gecko | Structured | 3 | 13 | 3.7944 | 0.3906 | 0.3835 |
| Gecko | Structured | 3 | 14 | 3.5137 | 0.3906 | 0.4282 |
| Gecko | Structured | 3 | 15 | 4.6538 | 0.3906 | 0.6788 |
| Gecko | Structured | 3 | 16 | 4.7940 | 0.3906 | 0.6735 |
| Gecko | Structured | 3 | 17 | 4.5714 | 0.3906 | 0.6700 |
| Gecko | Structured | 3 | 18 | 4.7441 | 0.3906 | 0.6840 |
| Gecko | Structured | 3 | 19 | 4.2317 | 0.3906 | 0.4196 |
| Gecko | Structured | 3 | 20 | 4.1860 | 0.4688 | 0.4106 |
| Gecko | Structured | 3 | 21 | 4.3805 | 0.3906 | 0.6729 |
| Gecko | Structured | 3 | 22 | 5.2061 | 0.3906 | 0.6801 |
| Gecko | Structured | 3 | 23 | 3.3146 | 0.3906 | 0.4489 |
| Gecko | Structured | 3 | 24 | 3.6363 | 0.3906 | 0.4986 |
| Gecko | Structured | 3 | 25 | 4.5731 | 0.3906 | 0.6785 |
| Gecko | Structured | 3 | 26 | 4.8285 | 0.3906 | 0.7024 |
| Gecko | Structured | 3 | 27 | 4.6383 | 0.3906 | 0.6778 |
| Gecko | Structured | 3 | 28 | 4.4198 | 0.3906 | 0.6782 |
| Gecko | Structured | 3 | 29 | 2.8458 | 0.2344 | 0.4247 |
| Gecko | Structured | 3 | 30 | 4.5108 | 0.3906 | 0.7117 |
