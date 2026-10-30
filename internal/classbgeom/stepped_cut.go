package classbgeom

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/units"
)

// This file is the crossing reach's stepped target
// (agent-docs/general-boolean-design.md §3 "Revolved shaft as a prism"): X is
// a stack of coaxial circular prisms, a stepped turned shaft, each step's Z1
// the next one's Z0. X's faces are its two end caps, one shoulder at each
// step boundary (the annulus between the two steps' circles), and one
// cylinder wall per step. Its carriers are the build's X carriers: the wall
// of step k is index k, the end caps are −1 and −2 as for a prism, and the
// shoulder at the start of step i (i ≥ 1) is −(2+i).

// CutSteppedView is class B's Cut of a stepped shaft x, read as its stack
// of coaxial circular prisms steps, by a prism tool y. A contained through
// tool along the shaft axis takes the exact axial-bore route first. Otherwise
// the perpendicular pair's entry gate
// runs on the stack's envelope, the circle of the largest radius swept over
// the whole span: B1–B8 read only its end caps, its circular wall and its
// span, and each of those bounds every step, so a pair the envelope admits
// meets no step in a way the gate refuses. Beside that gate the build
// misses when a straight tool wall across the shaft's axis lies strictly
// inside the whole span (BuildYCrossingFaces under XSlice), and B8 misses one
// in an end plane, so every such wall lies strictly outside the span, as
// CutPrismView's gate requires of a plain shaft; the shoulders, strictly
// inside the span, then lie in no tool plane. ok=false (err nil) is a silent
// miss.
func CutSteppedView(ctx context.Context, steps []prismrecord.Payload, y topo.Payload) (topo.Payload, bool, error) {
	envelope, ok := steppedEnvelope(steps)
	if !ok {
		return nil, false, nil
	}
	if payload, ok, err := cutSteppedBore(ctx, steps, y); err != nil || ok {
		return payload, ok, err
	}
	cp, ok, err := AdmitPair(ctx, envelope, y, envelope.Xform, 0)
	if err != nil || !ok {
		return nil, false, err
	}
	b, ok, err := newCBBuild(ctx, envelope, cp)
	if err != nil || !ok {
		return nil, false, err
	}
	b.steps = steps
	bp, err := b.run(ctx)
	if err == errCBMiss {
		return nil, false, nil
	}
	if err != nil {
		return nil, false, err
	}
	return bp, true, nil
}

// steppedEnvelope checks the stack's shape and states its envelope: two or
// more steps sharing one frame and placement, each one whole counter-clockwise
// circle about one centre with no displacement, each step's Z1 the next one's
// Z0 and every span positive.
func steppedEnvelope(steps []prismrecord.Payload) (prismrecord.Payload, bool) {
	if len(steps) < 2 {
		return prismrecord.Payload{}, false
	}
	first := steps[0]
	var widest sectionrecord.CircleSeg
	var widestR float64
	for i, s := range steps {
		c, ok := steppedCircle(s)
		if !ok || s.Frame != first.Frame || s.Xform != first.Xform || !(s.Z0 < s.Z1) ||
			s.Z0Delta != 0 || s.Z1Delta != 0 || s.SectionDelta != 0 || s.SurfaceResult {
			return prismrecord.Payload{}, false
		}
		if c.Center != steppedCenter(first) {
			return prismrecord.Payload{}, false
		}
		if i > 0 && steps[i-1].Z1 != s.Z0 {
			return prismrecord.Payload{}, false
		}
		r, err := c.Radius.In(units.Millimeter)
		if err != nil {
			return prismrecord.Payload{}, false
		}
		if i > 0 {
			prev, _ := steppedCircle(steps[i-1])
			pr, err := prev.Radius.In(units.Millimeter)
			if err != nil || pr == r {
				return prismrecord.Payload{}, false
			}
		}
		if r > widestR {
			widest, widestR = c, r
		}
	}
	return prismrecord.Payload{
		Profile: momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{widest}}},
		Frame:   first.Frame,
		Z0:      first.Z0,
		Z1:      steps[len(steps)-1].Z1,
		Xform:   first.Xform,
	}, true
}

// steppedCircle is a step's one whole counter-clockwise circle.
func steppedCircle(s prismrecord.Payload) (sectionrecord.CircleSeg, bool) {
	if len(s.Profile.Holes) != 0 || len(s.Profile.Outer.Segments) != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	c, ok := s.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 {
		return sectionrecord.CircleSeg{}, false
	}
	return c, true
}

func steppedCenter(s prismrecord.Payload) sectionrecord.Point2 {
	c, _ := steppedCircle(s)
	return c.Center
}

// cbShoulder is the carrier of the shoulder at the start of step i ≥ 1.
func cbShoulder(i int) cbCarrier { return cbCarrier{op: cbX, idx: -(2 + i)} }

// cbShoulderStep is the step a shoulder carrier starts.
func cbShoulderStep(c cbCarrier) int { return -c.idx - 2 }

// stepBelow and stepAbove are the planar carriers bounding step k's wall:
// an end cap or a shoulder.
func (b *cbBuild) stepBelow(k int) cbCarrier {
	if k == 0 {
		return cbCarrier{op: cbX, idx: -1}
	}
	return cbShoulder(k)
}

func (b *cbBuild) stepAbove(k int) cbCarrier {
	if k == len(b.steps)-1 {
		return cbCarrier{op: cbX, idx: -2}
	}
	return cbShoulder(k + 1)
}

func (b *cbBuild) stepRadius(k int) (float64, error) {
	c, _ := steppedCircle(b.steps[k])
	return c.Radius.In(units.Millimeter)
}

// buildSteppedXFaces decides a stepped X's planar faces against Y: each end
// cap, the disk of its end step, and each shoulder, the annulus between the
// two steps' circles, whose outer circle is the wider step's wall and whose
// hole the narrower one's. A shoulder faces away from the wider step. Each
// face's scene is §5's, against Y's chord in the face's plane swept between
// Y's caps, exactly as a prism's cap is.
func (b *cbBuild) buildSteppedXFaces(ctx context.Context, hooks CrossingFaceHooks[cbCarrier]) error {
	n := len(b.steps)
	frame := CrossingFrame{Frame: b.fX.frame, Axis: b.fX.axis, Sign: b.fX.sign}
	wall := func(k int) cbSeg {
		return cbSeg{Seg: b.steps[k].Profile.Outer.Segments[0], Carrier: cbCarrier{op: cbX, idx: k}}
	}
	hole := func(k int) cbSeg {
		c, _ := steppedCircle(b.steps[k])
		c.CCW, c.TStart, c.TEnd = false, 1, 0
		return cbSeg{Seg: c, Carrier: cbCarrier{op: cbX, idx: k}}
	}
	for i := 0; i <= n; i++ {
		var c cbCarrier
		var level float64
		var own []cbSeg
		var holes [][]cbSeg
		var outward bool
		switch i {
		case 0:
			c, level, own = cbCarrier{op: cbX, idx: -1}, b.steps[0].Z0, []cbSeg{wall(0)}
		case n:
			c, level, own, outward = cbCarrier{op: cbX, idx: -2}, b.steps[n-1].Z1, []cbSeg{wall(n - 1)}, true
		default:
			below, err := b.stepRadius(i - 1)
			if err != nil {
				return err
			}
			above, err := b.stepRadius(i)
			if err != nil {
				return err
			}
			c, level = cbShoulder(i), b.steps[i].Z0
			wide, narrow := i, i-1
			if below > above {
				wide, narrow, outward = i-1, i, true
			}
			own, holes = []cbSeg{wall(wide)}, [][]cbSeg{{hole(narrow)}}
		}
		chords, err := hooks.Chords(ctx, cbY, c)
		if err != nil {
			return err
		}
		other, err := ChordRectangles(frame, chords, cbY, hooks.Carrier, hooks.Geometry, hooks.Miss)
		if err != nil {
			return err
		}
		if err := b.decide(ctx, c, b.fX, level, outward, own, holes, other, true); err != nil {
			return err
		}
	}
	return nil
}

// steppedSlice is a stepped X's slice in the plane of Y's face f, in frame:
// the union of each step's chord in that plane swept over the step's span.
// Each step's chord is its circle's trace through sketch, as a prism's is,
// with its ends pinned at the step's two bounding planes. The steps whose
// circle the plane meets must be consecutive, so the slice is one staircase
// loop: up the chords' high ends, each step's wall joined to the next by the
// shoulder between them, across the top step's bounding plane, and down the
// low ends. Steps apart from the plane on both sides of a step that meets it
// would cut the slice into several loops, and the pair misses. No step
// meeting the plane gives no loop.
func (b *cbBuild) steppedSlice(ctx context.Context, f cbCarrier, frame CrossingFrame) ([][]cbSeg, error) {
	axis, level := b.geom(f).axis, b.geom(f).level
	if axis == 2 {
		return nil, errCBMiss
	}
	free := 1 - axis
	geometry := func(c cbCarrier) (int, float64) {
		g := b.geom(c)
		return g.axis, g.level
	}
	fX := CrossingFrame{Frame: b.fX.frame, Axis: b.fX.axis, Sign: b.fX.sign}
	type chord struct {
		k      int
		lo, hi float64
	}
	var chords []chord
	for k := range b.steps {
		carrier := func(_, idx int) cbCarrier {
			switch idx {
			case -1:
				return b.stepBelow(k)
			case -2:
				return b.stepAbove(k)
			}
			return cbCarrier{op: cbX, idx: k}
		}
		traced, err := TraceCrossingChords(ctx, cbX, f, b.steps[k].Profile.Outer.Segments, fX,
			carrier, geometry,
			func(f, c1, c2 cbCarrier, at [3]float64, delta float64) ([3]float64, error) {
				v, err := b.vertices.CanonicalPoint(f, c1, c2, at, delta)
				return v.Point, err
			}, errCBMiss)
		if err != nil {
			return nil, err
		}
		switch len(traced) {
		case 0:
			continue
		case 1:
		default:
			return nil, errCBMiss
		}
		lo, hi := traced[0].Lo[free], traced[0].Hi[free]
		if lo > hi {
			lo, hi = hi, lo
		}
		if !(lo < hi) {
			return nil, errCBMiss
		}
		// The steps meeting the plane must be consecutive.
		if len(chords) > 0 && chords[len(chords)-1].k != k-1 {
			return nil, errCBMiss
		}
		chords = append(chords, chord{k: k, lo: lo, hi: hi})
	}
	if len(chords) == 0 {
		return nil, nil
	}
	at := func(v, z float64) [3]float64 {
		var p [3]float64
		p[free], p[axis], p[2] = v, level, z
		return p
	}
	var corners [][3]float64
	var carriers []cbCarrier
	for i, c := range chords {
		if i > 0 && c.hi == chords[i-1].hi {
			return nil, errCBMiss
		}
		corners = append(corners, at(c.hi, b.steps[c.k].Z0), at(c.hi, b.steps[c.k].Z1))
		carriers = append(carriers, cbCarrier{op: cbX, idx: c.k}, b.stepAbove(c.k))
	}
	// The last riser above runs across the top step's bounding plane to the
	// low end; the walk then comes down the low ends.
	for i, c := range slices.Backward(chords) {
		if i < len(chords)-1 && c.lo == chords[i+1].lo {
			return nil, errCBMiss
		}
		corners = append(corners, at(c.lo, b.steps[c.k].Z1), at(c.lo, b.steps[c.k].Z0))
		carriers = append(carriers, cbCarrier{op: cbX, idx: c.k}, b.stepBelow(c.k))
	}
	return [][]cbSeg{RectLoop(frame.Axis, frame.Sign, corners, carriers)}, nil
}

// loopsApart reports whether every loop of loops is box-apart from every
// loop of other, by exact comparison of the segments' outward boxes. Two
// loops whose boxes are apart share no point.
func loopsApart(loops, other [][]cbSeg) (bool, error) {
	box := func(loop []cbSeg) (Box2, error) {
		var out Box2
		for i, s := range loop {
			b, err := SegmentBox(s.Seg)
			if err != nil {
				return Box2{}, err
			}
			if i == 0 {
				out = b
				continue
			}
			out = Union(out, b)
		}
		return out, nil
	}
	for _, h := range loops {
		hb, err := box(h)
		if err != nil {
			return false, err
		}
		for _, o := range other {
			ob, err := box(o)
			if err != nil {
				return false, err
			}
			if !Apart(hb.Lo[0], hb.Hi[0], ob.Lo[0], ob.Hi[0]) && !Apart(hb.Lo[1], hb.Hi[1], ob.Lo[1], ob.Hi[1]) {
				return false, nil
			}
		}
	}
	return true, nil
}

// decideAroundHoles decides a holed face whose holes are apart from the
// other operand's slice (loopsApart): its outer loop's scene alone, then the
// holes added to the one face that scene keeps. The caller's holes lie
// strictly inside the outer loop (a shoulder's narrower circle, concentric
// with the wider one), and each is apart from the slice, so each hole's disk
// lies inside the kept area and its circle touches none of the kept area's
// boundary. A connected disk lies in one kept face; with more than one kept
// face the pair misses rather than place it, and with none the hole has
// nowhere to lie, which the caller's geometry rules out, so that misses too.
func (b *cbBuild) decideAroundHoles(ctx context.Context, f cbCarrier, frame cbFrame, level float64, outward bool,
	own []cbSeg, ownHoles [][]cbSeg, other [][]cbSeg, cut bool) error {
	if !cut {
		return errCBMiss
	}
	before := len(b.faces)
	if err := b.decide(ctx, f, frame, level, outward, own, nil, other, cut); err != nil {
		return err
	}
	if len(b.faces) != before+1 {
		return errCBMiss
	}
	face := &b.faces[before]
	for _, h := range ownHoles {
		var loop sectionrecord.LoopRecord
		row := make([]cbCarrier, len(h))
		for i, s := range h {
			loop.Segments = append(loop.Segments, s.Seg)
			row[i] = s.Carrier
		}
		face.region.Holes = append(face.region.Holes, loop)
		face.carriers = append(face.carriers, row)
	}
	return nil
}
