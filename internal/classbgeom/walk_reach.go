package classbgeom

import (
	"context"
	"sort"
)

// Walk is the tool's sweep read through every target face its tube meets
// (agent-docs/general-boolean-design.md §3 "Pin across several faces"), in
// the frame g: the tool spans [A, B] along g's normal, and Slabs lists the
// faces in strictly increasing Level order.
type Walk struct {
	Slabs []WalkFace
	A, B  float64
	// Inside[k] says the target holds layer k of the tube: below the first
	// face for k = 0, between faces k and k+1, and above the last face for
	// k = len(Slabs).
	Inside []bool
}

// WalkFace is one target face across the sweep, read in g.
type WalkFace struct {
	Slab
	// GLevel is the face's level along g's normal, and Up says its outward
	// normal is g's +N.
	GLevel float64
	Up     bool
}

// LowFlush reports that the first face lies at the tool's cap A, so the
// tube's interval below it is empty.
func (w Walk) LowFlush() bool { return w.Slabs[0].GLevel == w.A }

// HighFlush reports that the last face lies at the tool's cap B.
func (w Walk) HighFlush() bool { return w.Slabs[len(w.Slabs)-1].GLevel == w.B }

// Interval returns layer k's part of the tool's interval, and whether it is
// nonempty.
func (w Walk) Interval(k int) (float64, float64, bool) {
	lo, hi := w.A, w.B
	if k > 0 {
		lo = w.Slabs[k-1].GLevel
	}
	if k < len(w.Slabs) {
		hi = w.Slabs[k].GLevel
	}
	return lo, hi, lo < hi
}

// WalkReach walks the tool's sweep through every target face whose box meets
// its tube. It admits the pair (W1–W4 of the design's gate) when every such
// face lies across the sweep, every face level and both tool caps carry no
// displacement, the tool spans a nonempty interval, the faces sit at strictly
// increasing levels, and consecutive faces point opposite ways along the
// sweep. Each face's outward normal then says which layers of the tube the
// target holds. The caller still proves each face's region holds the tool's
// section (W5) and checks the operation (W6). ok=false is a silent miss.
func WalkReach(ctx context.Context, faces []FaceRecord, tool ToolRecord) (Walk, bool, error) {
	slabs, ok, err := tubeSlabs(ctx, faces, tool, 0, 0)
	if err != nil || !ok {
		return Walk{}, false, err
	}
	if tool.Z0Delta != 0 || tool.Z1Delta != 0 || !(tool.Z0 < tool.Z1) {
		return Walk{}, false, nil
	}
	w := Walk{A: tool.Z0, B: tool.Z1, Slabs: make([]WalkFace, len(slabs))}
	for i, s := range slabs {
		if s.LevelDelta != 0 {
			return Walk{}, false, nil
		}
		w.Slabs[i] = WalkFace{Slab: s, GLevel: tool.Sign[2]*s.Level + 0, Up: float64(s.Outward)*tool.Sign[2] > 0}
	}
	sort.SliceStable(w.Slabs, func(i, j int) bool { return w.Slabs[i].GLevel < w.Slabs[j].GLevel })
	for i := 1; i < len(w.Slabs); i++ {
		if !(w.Slabs[i-1].GLevel < w.Slabs[i].GLevel) || w.Slabs[i-1].Up == w.Slabs[i].Up {
			return Walk{}, false, nil
		}
	}
	// The layer below a face is inside exactly when the face points up; the
	// layer above it, when it points down. W4 makes the readings agree.
	w.Inside = make([]bool, len(w.Slabs)+1)
	w.Inside[0] = w.Slabs[0].Up
	for i, s := range w.Slabs {
		w.Inside[i+1] = !s.Up
	}
	return w, true, nil
}
