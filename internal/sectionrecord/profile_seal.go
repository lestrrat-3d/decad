package sectionrecord

// AnalyticProfileView is a plane section's recorded loops at a seal boundary.
// The holder must snapshot the loops before retaining the view.
type AnalyticProfileView struct {
	Outer LoopRecord
	Holes []LoopRecord
}

// AnalyticWalkView stores the bitwise fields of one line or circular walk.
type AnalyticWalkView struct {
	Kind   uint8
	Closed bool
	Values [29]uint64
}

// AnalyticWalkSource holds a producer-bound identity and its analytic walks.
// The holder must snapshot the slices before retaining this view.
type AnalyticWalkSource struct {
	Anchor  *LoopRecord
	Profile AnalyticProfileView
	Outer   []AnalyticWalkView
	Holes   [][]AnalyticWalkView
}
