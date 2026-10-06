package cappatch

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
}
