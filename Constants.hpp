// The lava level.
// Don't change this unless you want the lava 
// to be somewhere else for some reason.
const double lava_y = -3071.0;

// The directions the platform can tilt from a PU position.
// Only change this if you want to limit the search to particular tilts.
const int n_normal_offsets = 4;
const float normal_offsets[n_normal_offsets][3] = { {0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, -0.01f}, {0.01f, -0.01f, -0.01f} };

// The directions the platform can tilt before PU movement.
// You may want to remove the +ny tilts if your second frame
// y normal is already quite low.
const int n_pre_tilt_offsets = 8;
const float pre_tilt_offsets[n_pre_tilt_offsets][3] = { {0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, 0.01f}, {-0.01f, -0.01f, -0.01f}, {0.01f, -0.01f, -0.01f}, {0.01f, 0.01f, 0.01f}, {-0.01f, 0.01f, 0.01f}, {-0.01f, 0.01f, -0.01f}, {0.01f, 0.01f, -0.01f} };

// The positions of the pyramid platforms.
// These are the locations of the two that are availabile in BitFS.
// Only change this if you want a different platform setup for some reason.
const short platform_positions[2][3] = { {-1945, -3225, -715}, {-2866, -3225, -715} };

// Positions of the y ranges where floor exists.
// These are the locations of the floors above the pole in BitFS.
// Only change these if you want to change the floor locations for some reason.
const int n_floor_ranges = 9;
const double lower_floor[n_floor_ranges] = { -1357.0, -743.0, 127.0, 675.0, 2687.0, 4325.0, 4940.0, 5170.0, 5400.0 };
const double upper_floor[n_floor_ranges] = { -1125.0, -665.0, 410.0, 1331.0, 3789.0, 4506.0, 5018.0, 5248.0, 5478.0 };