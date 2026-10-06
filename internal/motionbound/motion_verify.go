package motionbound

// MotionKind names which Motion variant a motionSpec was read from.
type MotionKind int

const (
	MotionRevolute MotionKind = iota + 1
	MotionPrismatic
	MotionBetween
)
