package capband

// Point is one plane-local cap patch coordinate.
type Point struct{ U, V float64 }

// Patch carries the geometric values consumed by cap band measurements.
type Patch struct {
	Circular                             bool
	SideA, SideB, CapA, CapB             Point
	CU, CV                               float64
	SideRadius, CapRadius                float64
	Th0, Th1                             float64
	SweepCCW, WholeTurn                  bool
	CapTh0, CapTh1                       float64
	SideZ, CapZ                          float64
	ContourAllow, LevelDelta, CapThAllow float64
	// SkewStart and SkewEnd are proven upper bounds, in radians, on the exact
	// angle about (CU, CV) between the side directrix's end and the cap
	// directrix's end at the window's start (Th0, CapTh0) and end (Th1,
	// CapTh1) corner (CornerSkewUpper). Both are zero on an apex patch, whose
	// side directrix is the single point the corner is, and wherever the two
	// ends lie on one ray from the centre. The Cone arm of AreaOf reads them.
	SkewStart, SkewEnd float64
}
