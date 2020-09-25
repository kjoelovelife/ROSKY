# intrinsic_calibration.launch

 Chessboard Options:
    You must specify one or more chessboards as pairs of --size and
    --square options.

    -p PATTERN, --pattern=PATTERN
                        calibration pattern to detect - 'chessboard',
                        'circles', 'acircles'
    -s SIZE, --size=SIZE
                        chessboard size as NxM, counting interior corners
                        (e.g. a standard chessboard is 7x7)
    -q SQUARE, --square=SQUARE
                        chessboard square size in meters

  ROS Communication Options:
    --approximate=APPROXIMATE
                        allow specified slop (in seconds) when pairing images
                        from unsynchronized stereo cameras
    --no-service-check  disable check for set_camera_info services at startup
    --queue-size=QUEUE_SIZE
                        image queue size (default 1, set to 0 for unlimited)

  Calibration Optimizer Options:
    --fix-principal-point
                        for pinhole, fix the principal point at the image
                        center
    --fix-aspect-ratio  for pinhole, enforce focal lengths (fx, fy) are equal
    --zero-tangent-dist
                        for pinhole, set tangential distortion coefficients
                        (p1, p2) to zero
    -k NUM_COEFFS, --k-coefficients=NUM_COEFFS
                        for pinhole, number of radial distortion coefficients
                        to use (up to 6, default 2)
    --fisheye-recompute-extrinsicsts
                        for fisheye, extrinsic will be recomputed after each
                        iteration of intrinsic optimization
    --fisheye-fix-skew  for fisheye, skew coefficient (alpha) is set to zero
                        and stay zero
    --fisheye-fix-principal-point
                        for fisheye,fix the principal point at the image
                        center
    --fisheye-k-coefficients=NUM_COEFFS
                        for fisheye, number of radial distortion coefficients
                        to use fixing to zero the rest (up to 4, default 4)
    --fisheye-check-conditions
                        for fisheye, the functions will check validity of
                        condition number
    --disable_calib_cb_fast_check
                        uses the CALIB_CB_FAST_CHECK flag for
                        findChessboardCorners
    --max-chessboard-speed=MAX_CHESSBOARD_SPEED
                        Do not use samples where the calibration pattern is
                        moving faster                      than this speed in
                        px/frame. Set to eg. 0.5 for rolling shutter cameras.


