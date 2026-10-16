# References (full entries: docs/references.md at the repository root):
#   [R1] TrackBotGP rules 2026: track features reproduced in the synthetic images
# More details: detector.py (the code under test); README.md of this package

import os

import pytest
import numpy as np

from learn2race_camera_follower.detector import DetectorConfig, LineDetector


def Config(**overrides):
    """The synthetic pictures below are drawn around the steering row of
    the earlier geometry (target row ~179 of 240, curve gain 0.55). They test
    the detection logic, not the tuning, so they keep that geometry."""
    values = dict(target_line_distance_mm=125.0, curve_gain=0.55)
    values.update(overrides)
    return DetectorConfig(**values)


def make_image(width: int, height: int, line_x: int | None) -> np.ndarray:
    image = np.full((height, width), 255, dtype=np.uint8)
    if line_x is not None:
        half_width = 8
        left = max(0, line_x - half_width)
        right = min(width, line_x + half_width)
        image[:, left:right] = 0
    return image


def test_centered_line_detects_small_error():
    detector = LineDetector(Config())
    image = make_image(320, 240, 160)
    result = detector.detect(image)

    assert result.found is True
    assert abs(result.center_error) < 0.1


def test_right_shifted_line_detects_positive_error():
    detector = LineDetector(Config())
    image = make_image(320, 240, 240)
    result = detector.detect(image)

    assert result.found is True
    assert result.center_error > 0.2


def test_missing_line_is_reported():
    detector = LineDetector(Config())
    image = make_image(320, 240, None)
    result = detector.detect(image)

    assert result.found is False
    # A blank floor has no contrast, which the adaptive threshold reports
    # explicitly rather than splitting noise into a spurious line.
    assert result.state in {"line_lost", "line_too_small", "low_contrast"}


def test_target_row_lies_inside_the_roi():
    """The distance-to-row mapping must land inside the region of interest.

    The original mapping ran the wrong way: image row 0 is the top of the frame
    and therefore the farthest point, so a short target distance produced a row
    near the top, above the ROI, which clamped to its first row. The parameter
    then had no effect at all. This asserts that two different target distances
    produce two different rows.
    """
    near = LineDetector(Config(target_line_distance_mm=60.0))
    far = LineDetector(Config(target_line_distance_mm=200.0))
    image = make_image(320, 240, 160)

    near_row = near.detect(image).target_row_roi
    far_row = far.detect(image).target_row_roi

    assert near_row != far_row, "target_line_distance_mm has no effect"
    # Nearer target means lower in the frame, so a larger row index.
    assert near_row > far_row


def test_wide_band_is_not_followed_as_a_line():
    """A finish line is a wide transverse band across the line and must not
    steer the robot. (A dark slab with no line through it is not one: since
    2026-10-02 a band must be a cross, the line continuing below and above.)"""
    detector = LineDetector(Config())
    image = np.full((240, 320), 255, dtype=np.uint8)
    image[:, 145:175] = 0                     # the track line
    image[165:195, 20:300] = 0                # the finish stripe across it

    result = detector.detect(image)

    assert result.band_detected is True
    assert result.found is False


def test_sharp_curve_is_tracked_not_rejected_as_a_band():
    """A diagonal stripe must be followed, not classified as a finish line.

    Classifying candidates by bounding-box width rejected this case: a curve
    crossing the region of interest diagonally has a bounding box spanning most
    of the image width, exactly like a transverse finish line, so the robot lost
    the line on every sharp curve. Classification now uses the columns covered
    within the band of rows around the target row, where a curve is narrow.
    """
    import cv2

    detector = LineDetector(Config())
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (20, 239), (300, 144), 0, thickness=16)

    result = detector.detect(image)

    assert result.found is True, f"curve rejected, state={result.state}"
    assert result.band_detected is False


# Geometry that puts the target row at about row 147 of 240, which the
# synthetic images below are drawn around.
ROW_147 = Config(image_start_distance_mm=40.0)


def make_fork(width=320, height=240):
    """A Y: one stripe splitting into a left and a right branch going up."""
    import cv2
    image = np.full((height, width), 255, dtype=np.uint8)
    cx, split = width // 2, int(height * 0.95)
    cv2.line(image, (cx, height - 1), (cx, split), 0, 14)
    cv2.line(image, (cx, split), (cx - 110, 0), 0, 14)
    cv2.line(image, (cx, split), (cx + 110, 0), 0, 14)
    return image


def test_fork_takes_preferred_branch():
    # Committed (the pit approach): the preferred branch. Otherwise straight
    # on wins where one branch is clearly straighter.
    image = make_fork()
    left = LineDetector(Config()).detect(image, preference="left", commit=True)
    right = LineDetector(Config()).detect(image, preference="right", commit=True)

    assert left.found and right.found
    assert left.fork_detected and left.branch_count == 2
    assert left.center_error < -0.1
    assert right.center_error > 0.1


def test_fork_without_preference_stays_near_previous_line():
    detector = LineDetector(Config())
    detector.last_line_x = 100.0
    result = detector.detect(make_fork())

    assert result.fork_detected
    assert result.center_error < 0.0


def test_finish_line_across_track_is_a_band():
    image = make_image(320, 240, 160)
    image[172:188, 20:300] = 0      # default geometry: continuation above and below
    assert LineDetector(Config()).detect(image).band_detected is True
    image = make_image(320, 240, 160)
    image[140:156, 20:300] = 0      # across the target rows (about row 147)
    result = LineDetector(ROW_147).detect(image)

    assert result.band_detected is True


def test_sharp_corner_is_not_a_band():
    # The line comes up the middle and turns hard right: wide, but one-sided.
    image = np.full((240, 320), 255, dtype=np.uint8)
    image[140:240, 152:168] = 0
    image[140:156, 152:320] = 0
    result = LineDetector(ROW_147).detect(image)

    assert result.band_detected is False
    assert result.found is True


def test_tight_curve_leaving_the_side_steers_hard_that_way():
    # The line comes up from the bottom centre and leaves through the right
    # edge below the target row (about row 147).
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (160, 239), (175, 200), 0, 14)
    cv2.line(image, (175, 200), (319, 175), 0, 14)
    result = LineDetector(ROW_147).detect(image)

    assert result.found is True
    assert result.state == "curve_exit"
    assert result.center_error > 0.95


def test_s_bend_crossing_the_view_is_followed_not_a_band():
    # Stripe comes up on the left, runs across the target rows, and leaves
    # upwards on the right: an S bend, not a finish line.
    image = np.full((240, 320), 255, dtype=np.uint8)
    image[180:240, 30:46] = 0
    image[172:188, 30:290] = 0      # across the default target row (~179)
    image[0:180, 274:290] = 0
    result = LineDetector(Config()).detect(image)

    assert result.band_detected is False
    assert result.found is True
    assert result.state == "s_bend"
    assert result.center_error > 0.5        # aims at the continuation, right


def test_curve_leaving_the_side_above_target_row_turns_hard():
    # As on the track: the stripe crosses the target row (~179) left of centre
    # and bends out through the left edge a little farther ahead.
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (0, 239), (130, 90), 0, 270, 360, 0, 40)
    result = LineDetector(Config()).detect(image)

    assert result.found is True
    assert result.center_error < 0.0
    assert result.curve_error < -0.3
    assert result.error < result.center_error - 0.2


def test_straight_line_has_no_curve_term():
    result = LineDetector(Config()).detect(make_image(320, 240, 200))
    assert abs(result.curve_error) < 0.05


def make_pit_fork():
    """Straight main line up the middle; the pit lane peels off to the left
    ahead of the target row (~179) and leaves through the left edge."""
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    image[:, 152:168] = 0
    cv2.line(image, (160, 160), (0, 90), 0, 16)
    return image


def test_pit_lane_peeling_off_does_not_pull_the_robot():
    result = LineDetector(Config()).detect(make_pit_fork())

    assert result.found is True
    assert result.fork_ahead is True
    assert abs(result.center_error) < 0.1
    assert abs(result.curve_error) < 0.1     # straight on, not into the pit


def test_pit_preference_steers_into_the_pit_lane():
    result = LineDetector(Config()).detect(make_pit_fork(), preference="left")

    assert result.fork_ahead is True
    assert result.curve_error < -0.3


def test_other_track_section_ahead_is_ignored():
    # A separate stripe high in the view (another part of the circuit) must
    # not bend the look-ahead of the line being followed.
    image = make_image(320, 240, 160)
    image[80:96, 0:120] = 0
    result = LineDetector(Config()).detect(image)

    assert abs(result.curve_error) < 0.05


def test_default_tuning_steers_three_quarters_down():
    # Default steering row ~3/4 down the image (row ~179 of 240).
    result = LineDetector(DetectorConfig()).detect(make_image(320, 240, 200))

    assert result.found is True
    assert 150 < result.target_row_abs < 200
    assert result.center_error > 0.2
    assert abs(result.curve_error) < 0.05


def test_hairpin_arch_turns_hard_towards_the_returning_leg():
    # A right hairpin seen from its entry: our leg comes up on the left,
    # peaks, and the returning leg comes back down on the right.
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (160, 239), (60, 110), 0, 180, 360, 0, 22)
    result = LineDetector(Config()).detect(image)

    assert result.found is True
    assert result.state == "hairpin_right"
    assert result.error >= 0.99
    assert result.fork_detected is False     # not a pit fork for the referee


def test_dark_area_high_in_view_is_not_followed():
    # A wall across the top of the window that reaches the target row only
    # at one edge: not the line under the robot.
    image = make_image(320, 240, 160)
    image[72:180, 0:60] = 0
    detector = LineDetector(Config())
    detector.last_line_x = 40.0      # as if the line had been seen near it
    result = detector.detect(image)

    assert result.found is True
    assert abs(result.center_error) < 0.1


def test_hairpin_apex_seen_head_on_is_not_a_finish_line():
    # The top of a tight U-turn crossing the target row from side to side,
    # with both legs below it and nothing above.
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (160, 260), (120, 85), 0, 180, 360, 0, 24)
    result = LineDetector(Config()).detect(image)

    assert result.band_detected is False


def shadowed_floor(line_x=220):
    """White floor, the robot's soft shadow over the left half, a black line."""
    import cv2
    image = np.full((240, 320), 210, dtype=np.uint8)
    image[:, :150] = 125                                 # shadow: ~40% darker
    image = cv2.GaussianBlur(image, (41, 41), 0)         # soft shadow edge
    image[:, line_x - 9:line_x + 9] = 40                 # black track line
    return image


def test_robot_shadow_is_not_taken_for_the_line():
    # (otsu used to fail here; it is no longer the default.)
    detector = LineDetector(Config(threshold_mode="relative"))
    detector.last_line_x = 100.0          # as if last seen on the shadow side
    result = detector.detect(shadowed_floor())
    assert result.found and result.center_error > 0.25


def test_line_inside_the_shadow_is_still_found():
    image = shadowed_floor(line_x=80)
    image[:, 71:89] = 25                  # the line in shadow: darker still
    result = LineDetector(Config()).detect(image)

    assert result.found is True
    assert result.center_error < -0.35


def test_finish_line_is_detected_next_to_the_shadow():
    image = shadowed_floor(line_x=160)
    image[172:188, 20:300] = 40                         # finish stripe across
    result = LineDetector(Config()).detect(image)

    assert result.band_detected is True


def test_thick_finish_stripe_close_to_the_robot_is_detected():
    # Seen close up the stripe with its chequers is ~90 px tall.
    image = make_image(320, 240, 160)
    image[135:225, 10:310] = 0
    result = LineDetector(Config()).detect(image)

    assert result.band_detected is True


def test_pit_fork_is_detected_next_to_the_shadow():
    image = make_pit_fork()
    image[:, :60] = np.minimum(image[:, :60], 130)       # shadow on the left
    result = LineDetector(Config()).detect(image, preference="left")

    assert result.fork_ahead is True
    assert result.curve_error < -0.3                     # steers into the pit branch


def test_dark_shadow_merged_with_the_line_is_split_off():
    # Under one strong lamp the shadow is nearly as dark as the tape
    # (0.35 vs 0.15 of the floor) and touches it: one blob, two levels.
    image = np.full((240, 320), 200, dtype=np.uint8)
    image[150:240, 60:190] = 72                      # the robot's shadow
    image[:, 190:240] = 30                           # the line, touching it
    result = LineDetector(Config()).detect(image)

    assert result.found is True
    assert result.band_detected is False
    assert result.center_error > 0.25                # on the line (x~215), not the shadow


def test_shiny_streak_on_the_tape_is_not_split_off():
    # A narrow lighter streak inside the tape is part of the line.
    image = make_image(320, 240, 160)
    image[:, 152:168] = 35
    image[:, 163:168] = 95                           # shiny streak, ~0.4 of the floor
    result = LineDetector(Config()).detect(image)

    assert result.found is True
    assert abs(result.center_error) < 0.05


def finish_marker(square=8, top=120, rows=3, x0=40, x1=280):
    """The track's finish marker (photo 2026-10-01): a chequered strip of
    small squares with a solid black bar along its near edge, the track line
    running through it."""
    image = make_image(320, 240, 160)
    for r in range(rows):
        for c, x in enumerate(range(x0, x1, square)):
            image[top + r * square: top + (r + 1) * square, x:x + square] = 30 if (r + c) % 2 == 0 else 225
    bar = top + rows * square
    image[bar:bar + square + 4, x0:x1] = 30
    return image


def test_chequered_finish_marker_is_detected_near_and_far():
    for square, top in ((7, 90), (10, 130), (12, 150)):
        result = LineDetector(Config()).detect(finish_marker(square=square, top=top))

        assert result.checker_detected is True, (square, top)
        assert result.band_detected is True
        assert result.fork_detected is False     # its squares are not a pit fork


def test_parallel_lines_are_not_a_chequer():
    # Three parallel track sections in view: regular dark pieces in every row.
    image = np.full((240, 320), 225, dtype=np.uint8)
    for x in (60, 150, 240):
        image[:, x:x + 16] = 30
    assert LineDetector(Config()).detect(image).checker_detected is False


def test_pit_split_v_is_a_fork_not_a_band():
    """2026-10-02 00:28: the pit split, a V with its stem near the robot, was
    called a finish band, and the follower coasted straight past the pit."""
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (160, 240), (160, 0), 0, 30)          # main line
    cv2.line(image, (175, 240), (330, 40), 0, 30)         # pit branch to the right
    right = LineDetector(Config()).detect(image, "right", commit=True)   # pitting
    left = LineDetector(Config()).detect(image, "left")
    passing = LineDetector(Config()).detect(image, "right")             # not pitting
    assert right.found and not right.band_detected
    assert right.fork_detected
    assert right.error > left.error + 0.2
    assert passing.error < right.error - 0.2                           # main line, straight on


def test_one_sided_curve_is_not_a_band():
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (330, 240), (210, 120), 0, 180, 270, 0, 30)   # bottom-left to top-right edge
    result = LineDetector(Config()).detect(image)
    assert result.found and not result.band_detected


def test_committed_to_pit_branch_when_it_leaves_the_view():
    """2026-10-02 01:29: the pit branch slid out of the side of the image
    before the robot turned, and the follower went back to the main line."""
    import cv2
    v = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(v, (160, 240), (160, 0), 0, 30)
    cv2.line(v, (175, 240), (330, 40), 0, 30)
    main_only = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(main_only, (150, 240), (150, 0), 0, 30)

    committed = LineDetector(Config())
    assert committed.detect(v, "right", commit=True).fork_detected
    r = committed.detect(main_only, "right", commit=True)
    assert r.state == "fork_commit" and r.error >= 0.99

    free = LineDetector(Config())
    free.detect(v, "right")
    r = free.detect(main_only, "right")
    assert r.state != "fork_commit" and r.error < 0.2

    for _ in range(40):                          # the commitment runs out
        r = committed.detect(main_only, "right", commit=True)
    assert r.state != "fork_commit"


def test_commit_follows_the_branch_once_turned():
    """2026-10-02 01:39 (Monza): after turning towards the pit branch the
    branch moved left in the image and was taken for the main line; the
    robot kept turning past it. A lone line right of the main line's last
    position is the branch: follow it."""
    import cv2
    v = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(v, (20, 240), (20, 0), 0, 30)                # main line at the left edge
    cv2.line(v, (150, 240), (200, 0), 0, 30)              # pit branch, now ahead
    branch_only = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(branch_only, (130, 240), (170, 0), 0, 30)    # moved left as the robot turned
    d = LineDetector(Config())
    assert d.detect(v, "right", commit=True).fork_detected
    r = d.detect(branch_only, "right", commit=True)
    assert r.state != "fork_commit" and r.error < 0.5


def _x_crossing(y_cross, angle_deg=50):
    """Our line straight up the middle, another crossing it at y_cross."""
    import cv2
    import math
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (160, 240), (160, 0), 0, 26)
    dx = 400 * math.cos(math.radians(angle_deg))
    dy = 400 * math.sin(math.radians(angle_deg))
    cv2.line(image, (int(160 - dx), int(y_cross + dy)), (int(160 + dx), int(y_cross - dy)), 0, 26)
    return image


@pytest.mark.parametrize("y_cross", [110, 150, 179, 205])
@pytest.mark.parametrize("preference", [None, "left", "right"])
def test_crossing_goes_straight_across(y_cross, preference):
    """Suzuka is a figure of eight: at the crossing the follower must go
    straight on, not take the crossing line as a split or a hairpin."""
    r = LineDetector(Config()).detect(_x_crossing(y_cross), preference)
    assert r.found and not r.fork_detected
    assert not r.state.startswith("hairpin")
    assert abs(r.error) < 0.3, (r.state, r.error)


def test_border_ends_count_line_split_crossing():
    import cv2
    d = LineDetector(Config())
    line = np.zeros((150, 320), dtype=np.uint8)
    cv2.line(line, (160, 149), (160, 0), 255, 20)
    y = line.copy()
    cv2.line(y, (165, 149), (330, 20), 255, 20)
    x = line.copy()
    cv2.line(x, (0, 140), (319, 10), 255, 20)
    assert [len(d._border_ends(m)) for m in (line, y, x)] == [2, 3, 4]


def _y_from_crossing(dir_deg):
    """Near the robot a crossing's lower legs merge at the image edge: what is
    left looks like a split. Our line comes up at dir_deg from vertical."""
    import cv2
    import math
    image = np.full((240, 320), 255, dtype=np.uint8)
    t = math.tan(math.radians(dir_deg))
    cv2.line(image, (150, 240), (int(150 + 240 * t), 0), 0, 26)                  # our line, straight on
    cv2.line(image, (150, 240), (int(150 - 240 * math.tan(math.radians(40))), 0), 0, 26)   # the crossing line
    return image


@pytest.mark.parametrize("preference", [None, "left", "right"])
def test_straight_on_at_a_crossing_that_looks_like_a_split(preference):
    import cv2
    d = LineDetector(Config())
    approach = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(approach, (150, 240), (150 + int(240 * 0.18), 0), 0, 26)   # ~10 deg right of vertical
    for _ in range(5):
        d.detect(approach, preference)
    r = d.detect(_y_from_crossing(10), preference)
    assert r.found and r.line_x > 150          # stays on the line going up-right


def test_pit_commit_still_takes_the_preferred_branch():
    import cv2
    d = LineDetector(Config())
    approach = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(approach, (160, 240), (160, 0), 0, 26)
    for _ in range(5):
        d.detect(approach, "right", commit=True)
    v = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(v, (160, 240), (160, 0), 0, 30)
    cv2.line(v, (175, 240), (330, 40), 0, 30)
    r = d.detect(v, "right", commit=True)
    assert r.error > 0.3                        # into the pit branch, not straight on


def test_straight_through_x_right_after_a_curve():
    """2026-10-02 02:16 (Suzuka): the robot met the crossing just out of a
    curve, with no recent direction in memory, and the crossing line's lower
    leg ran into its own line at the image corner (3 ends, like a split);
    it took the preferred branch, the crossing line. The direction of its
    own leg below the junction in the same frame says straight on."""
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (110, 240), (95, 0), 0, 34)           # our line, nearly vertical
    cv2.line(image, (-20, 250), (330, 60), 0, 30)          # crossing line, lower-left to upper-right
    for preference in ("right", "left", None):
        r = LineDetector(Config()).detect(image, preference)   # fresh: no memory
        assert r.found and abs(r.line_x - 103) < 30, (preference, r.state, r.line_x)


def test_hairpin_reaching_the_top_edge_is_not_a_crossing():
    """2026-10-02 19:01 (Suzuka): the curve after the crossing was taken for
    a crossing (its arch reached the top of the view) and driven straight
    into. An arch's inside is floor: no straight way through."""
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (175, 120), (70, 125), 0, 180, 360, 0, 26)     # U, top at the edge
    cv2.line(image, (105, 120), (105, 240), 0, 26)
    cv2.line(image, (245, 120), (245, 240), 0, 26)
    r = LineDetector(Config()).detect(image, None)
    assert r.found and r.state != "crossing"


def test_s_bend_leaving_the_view_four_times_is_not_a_crossing():
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.ellipse(image, (330, 240), (170, 110), 0, 180, 270, 0, 26)    # bottom up to the right edge
    cv2.ellipse(image, (330, 0), (170, 70), 0, 90, 180, 0, 26)        # back in from the right, to the top
    d = LineDetector(Config())
    full = (image < 128).astype(np.uint8) * 255
    assert d._crossing_aim(full, 150) is None


def test_lit_end_of_the_tape_is_not_dropped_as_shadow():
    """2026-10-02 19:10: near the robot the track light glints on the tape,
    which looks lighter there; the shadow split dropped that part and the
    line ended above the steering row (false hairpins)."""
    image = np.full((240, 320), 200, dtype=np.uint8)
    image[:, 150:180] = 30                 # tape far away: dark
    image[150:, 150:180] = 70              # near the robot: lit, lighter
    r = LineDetector(Config()).detect(image, None)
    rows = np.flatnonzero(r.mask.any(axis=1))
    assert r.found and rows.max() >= r.mask.shape[0] - 3
    assert not r.state.startswith("hairpin")


def test_sharp_bend_ahead_is_reported():
    import cv2
    image = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(image, (150, 240), (150, 120), 0, 26)
    cv2.ellipse(image, (200, 120), (50, 40), 0, 180, 270, 0, 26)     # turns right ahead
    cv2.line(image, (200, 80), (330, 80), 0, 26)
    r = LineDetector(Config()).detect(image, None)
    assert r.bend_px is not None and r.bend_side == "right"
    straight = np.full((240, 320), 255, dtype=np.uint8)
    cv2.line(straight, (150, 240), (165, 0), 0, 26)
    assert LineDetector(Config()).detect(straight, None).bend_px is None


# The settings config/track_suzuka.yaml gives the follower (off on other tracks).
SUZUKA = dict(hairpin_latch_frames=30, merged_width_ratio=1.6,
              hairpin_after_crossing_frames=15, bend_after_crossing_frames=40)


SUZUKA_HAIRPIN = os.path.join(os.path.dirname(__file__), "data", "suzuka_hairpin_20261008")


def _suzuka_hairpin_errors(latch_frames):
    """Frames 3385-3415 (every 5th) of the right hairpin lost on Suzuka,
    2026-10-08 02:11, with the robot's own detector settings."""
    import cv2
    detector = LineDetector(DetectorConfig(**dict(SUZUKA, hairpin_latch_frames=latch_frames)))
    errors = []
    for name in sorted(os.listdir(SUZUKA_HAIRPIN)):
        gray = cv2.imread(os.path.join(SUZUKA_HAIRPIN, name), cv2.IMREAD_GRAYSCALE)
        errors.append(detector.detect(gray, "right").error)
    return errors


def test_suzuka_hairpin_without_latch_turns_away():
    # The recorded failure: the arch is mis-read and the robot turns hard left.
    assert _suzuka_hairpin_errors(0)[-1] == pytest.approx(-1.0)


def test_suzuka_hairpin_latch_keeps_turning_right():
    errors = _suzuka_hairpin_errors(40)
    assert errors[0] == pytest.approx(1.0)          # hairpin_right seen
    assert min(errors) >= 0.0                       # never steers away
    assert errors[-1] == pytest.approx(1.0)         # still turning into it


def test_hairpin_latch_releases_on_a_line_straight_ahead():
    import cv2
    detector = LineDetector(DetectorConfig(**SUZUKA))
    first = sorted(os.listdir(SUZUKA_HAIRPIN))[0]
    detector.detect(cv2.imread(os.path.join(SUZUKA_HAIRPIN, first), cv2.IMREAD_GRAYSCALE), "right")
    assert detector.hairpin_left > 0
    detector.detect(make_image(320, 240, 160))      # out of the hairpin: centred line ahead
    assert detector.hairpin_left == 0
    result = detector.detect(make_image(320, 240, 110))
    assert result.error < -0.1                      # left corrections allowed again


SUZUKA_PIT_EXIT = os.path.join(os.path.dirname(__file__), "data", "suzuka_pit_exit_20261008")


def test_hairpin_latch_holds_while_the_arch_is_in_view():
    # Leaving the pit box (Suzuka 2026-10-08 22:17, every frame 6300-6346): the
    # exit line lies across the robot as an arch for longer than the latch.
    # Counting down regardless, it expired at frame 6337 and the follower took
    # the left leg (-0.55) and lost the line.
    import cv2
    detector = LineDetector(DetectorConfig(**SUZUKA))
    errors = [detector.detect(cv2.imread(os.path.join(SUZUKA_PIT_EXIT, name), cv2.IMREAD_GRAYSCALE),
                              "left").error
              for name in sorted(os.listdir(SUZUKA_PIT_EXIT))]
    assert errors[0] == pytest.approx(1.0)
    assert min(errors) >= 0.0


SUZUKA_CROSSING = os.path.join(os.path.dirname(__file__), "data", "suzuka_crossing_20261008")


def test_hairpin_latch_does_not_carry_through_a_crossing():
    # Suzuka 2026-10-08 22:48, frames 12596-12608: met at an angle, the
    # crossing reads as a left hairpin for 4 frames (12600-12603), then as a
    # crossing again. Latched, the full left turn was held through it.
    import cv2
    detector = LineDetector(DetectorConfig(**SUZUKA))
    states = [detector.detect(cv2.imread(os.path.join(SUZUKA_CROSSING, name), cv2.IMREAD_GRAYSCALE),
                              "right").state
              for name in sorted(os.listdir(SUZUKA_CROSSING))]
    assert "crossing" in states[:4]
    assert states[-5:] == ["crossing"] * 5      # frames 12604-12608


SUZUKA_PIT_SPLIT = os.path.join(os.path.dirname(__file__), "data", "suzuka_pit_split_20261008")


@pytest.mark.parametrize("prefer_main, side", [(False, -1.0), (True, 1.0)])
def test_suzuka_pit_split_takes_the_main_line_when_asked(prefer_main, side):
    # Suzuka 2026-10-08 22:52, frames 4978-4992, lap 1 (preference "right",
    # the main line): the pit lane leaves straight on, to the left. Straight on
    # (the default) takes it; split_prefer_main takes the main line.
    import cv2
    detector = LineDetector(DetectorConfig(**dict(SUZUKA, split_prefer_main=prefer_main)))
    results = [detector.detect(cv2.imread(os.path.join(SUZUKA_PIT_SPLIT, name), cv2.IMREAD_GRAYSCALE),
                               "right")
               for name in sorted(os.listdir(SUZUKA_PIT_SPLIT))]
    forks = [r.error for r in results if r.fork_detected]
    assert forks and all(e * side > 0.1 for e in forks)


SUZUKA_PIT_MERGE = os.path.join(os.path.dirname(__file__), "data", "suzuka_pit_merge_20261008")


def test_merging_line_does_not_pull_the_follower_off_its_line():
    # Suzuka 2026-10-08 23:01, frames 5900-5950: just past the finish line the
    # pit lane joins the main line from the left. Touching on the steering row,
    # both made one 158 px run; steering at its middle took the pit exit
    # backwards and the robot ran the track the wrong way.
    import cv2
    detector = LineDetector(DetectorConfig(**SUZUKA))
    xs = [detector.detect(cv2.imread(os.path.join(SUZUKA_PIT_MERGE, name), cv2.IMREAD_GRAYSCALE),
                          "right").line_x
          for name in sorted(os.listdir(SUZUKA_PIT_MERGE))]
    assert min(x for x in xs[36:] if x is not None) > 115     # main line at x ~128-135, not ~91


SUZUKA_CROSSING_2 = os.path.join(os.path.dirname(__file__), "data", "suzuka_crossing_rotated_20261008")


def test_no_hairpin_turn_and_no_bend_on_top_of_a_crossing():
    # Suzuka 2026-10-08 23:08, frames 4876-4895: one frame of "hairpin left"
    # (+1.8 rad/s) on the X rotated the robot ~30 deg, and the crossing's arms
    # as a close bend slowed it to a pivot there. It left on the wrong arm.
    import cv2
    detector = LineDetector(DetectorConfig(**SUZUKA))
    results = [detector.detect(cv2.imread(os.path.join(SUZUKA_CROSSING_2, name), cv2.IMREAD_GRAYSCALE),
                               "right")
               for name in sorted(os.listdir(SUZUKA_CROSSING_2))]
    first_x = next(i for i, r in enumerate(results) if r.state == "crossing")
    after = results[first_x:]
    assert not any(r.state.startswith("hairpin") for r in after)
    assert all(abs(r.error) < 0.5 for r in after)
    assert all(r.bend_px is None for r in after)


DRAGSTER_FINISH_T = os.path.join(os.path.dirname(__file__), "data", "dragster_finish_t_20261009")


@pytest.mark.parametrize("t_end, finish", [(False, False), (True, True)])
def test_dragster_finish_t_is_a_band_in_dragster_mode(t_end, finish):
    # Dragster 2026-10-09, frames 2712-2724: the line ends in a T at the finish
    # line. Without the T rule the close bar read as a hairpin and the follower
    # took its left arm; with it (dragster mode) the T is the finish band.
    import cv2
    detector = LineDetector(DetectorConfig(band_t_end=t_end))
    results = [detector.detect(cv2.imread(os.path.join(DRAGSTER_FINISH_T, name), cv2.IMREAD_GRAYSCALE))
               for name in sorted(os.listdir(DRAGSTER_FINISH_T))]
    assert results[-1].band_detected is finish
    # Frames 2712-2718: line only, no bar yet. (2722 has the bar ahead: with the
    # T-ahead check that already counts.)
    assert not any(r.band_detected for r in results[:4])


DRAGSTER_FINISH_T_1MS = os.path.join(os.path.dirname(__file__), "data", "dragster_finish_t_1ms_20261009")


def test_dragster_finish_t_seen_ahead_at_speed():
    # Dragster at 1.0 m/s, 2026-10-09 23:47, every 4th frame 5868-5884: the T is
    # only ahead of the steering row in 5884 and past it by 5888. Seen ahead, it
    # counts; missed, the follower spun searching for 2 s.
    import cv2
    detector = LineDetector(DetectorConfig(band_t_end=True))
    bands = [detector.detect(cv2.imread(os.path.join(DRAGSTER_FINISH_T_1MS, name), cv2.IMREAD_GRAYSCALE)).band_detected
             for name in sorted(os.listdir(DRAGSTER_FINISH_T_1MS))]
    assert bands == [False, False, False, False, True]
