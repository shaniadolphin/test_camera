### 1. Load images and exposure times

```
vector<Mat> images;
vector<float> times;
loadExposureSeq(argv[1], images, times);
```

Firstly we load input images and exposure times from user-defined folder. The folder should contain images and *list.txt* - file that contains file names and inverse exposure times.

For our image sequence the list is following:

```
memorial00.png 0.03125
memorial01.png 0.0625
...
memorial15.png 1024
```

### 2. Estimate camera response

```
Mat response;
Ptr<CalibrateDebevec> calibrate = createCalibrateDebevec();
calibrate->process(images, response, times);
```

It is necessary to know camera response function (CRF) for a lot of HDR construction algorithms. We use one of the calibration algorithms to estimate inverse CRF for all 256 pixel values.

### 3. Make HDR image

```
Mat hdr;
Ptr<MergeDebevec> merge_debevec = createMergeDebevec();
merge_debevec->process(images, hdr, times, response);
```

We use Debevec’s weighting scheme to construct HDR image using response calculated in the previous item.

### 4. Perform exposure fusion

 ```
Mat fusion;
Ptr<MergeMertens> merge_mertens = createMergeMertens();
merge_mertens->process(images, fusion);
 ```

There is an alternative way to merge our exposures in case when we don’t need HDR image. This process is called exposure fusion and produces LDR image that doesn’t require gamma correction. It also doesn’t use exposure values of the photographs.

### 5. Write results

```
imwrite("fusion.png", fusion * 255);
imwrite("ldr.png", ldr * 255);
imwrite("hdr.hdr", hdr);
```

Now it’s time to look at the results. Note that HDR image can’t be stored in one of common image formats, so we save it to Radiance image (.hdr). Also all HDR imaging functions return results in [0, 1] range so we should multiply result by 255.