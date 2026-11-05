package parts

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/decad/internal/jointinfer"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PullSource says where the direction a part was pulled in came from.
type PullSource int

const (
	PullNone      PullSource = iota // the ground, or a part no joint reaches
	AxisFromFaces                   // the axis of the matched cylinder faces
	AxisDeclared                    // the joint's declared Axis
	FaceNormal                      // the normal of the matched contact faces
	BoxCentres                      // the line between the two parts' box centres
)

// String returns the lower-case words for the source.
func (p PullSource) String() string {
	switch p {
	case AxisFromFaces:
		return "axis from faces"
	case AxisDeclared:
		return "axis declared"
	case FaceNormal:
		return "face normal"
	case BoxCentres:
		return "box centres"
	}
	return "none"
}

// Exploded is the layout of a Set pulled apart along its joints.
type Exploded struct {
	Ground   string
	Parts    []ExplodedPart // in Set order, body-less parts omitted
	Leaders  []Leader
	Overlaps [][2]string // part pairs whose exploded boxes still meet
}

// ExplodedPart is one part's place in the exploded layout.
type ExplodedPart struct {
	Name      string
	Offset    r3.Vec       // translation after the assembled pose
	Placement r3.Transform // the same, as a transform
	Depth     int          // joint steps from the ground; -1 when no joint reaches it
	Anchor    r3.Vec       // label anchor: exploded box centre
	Pull      PullSource
}

// Leader is the dotted line one joint draws in the exploded view.
type Leader struct {
	Joint    string
	From, To r3.Vec // exploded coordinates: the port on A, the port on B
}

// reading is what the faces and the declaration say about one joint.
type reading struct {
	src    PullSource
	mode   explodeplan.Mode
	dir    r3.Vec
	axis   *Axis   // PinInBore and PinInSlot: the axis used
	length float64 // axial engagement, millimetres
	port   r3.Vec  // where the joint attaches, assembled coordinates
	portD  r3.Vec  // the axis or face normal at the port; zero when none
	facesA []*decad.Face
	facesB []*decad.Face
	// measured is the clearance read from the faces, when they matched.
	measured, bound float64
	matched         bool
	faceted         bool // a face that would have taken part reads as neither plane nor cylinder
}

func (s *Set) matchParams(c config) jointinfer.Params {
	return jointinfer.Params{MaxFit: c.maxFit, FitTol: c.fitTol, AxisTol: c.axisTol}
}

// readJoint reads joint j against the faces of its two parts (agent-docs/parts-design.md §7.1).
func (s *Set) readJoint(ctx context.Context, j Joint, fc *faceCache, c config) (reading, error) {
	pa, pb := s.parts[s.index[j.A]], s.parts[s.index[j.B]]
	var r reading
	boxA, boxB := toBox(pa.Body), toBox(pb.Body)
	if pa.Body != nil && pb.Body != nil {
		a, err := fc.get(ctx, pa.Body)
		if err != nil {
			return r, err
		}
		b, err := fc.get(ctx, pb.Body)
		if err != nil {
			return r, err
		}
		boxA, boxB = fromDecad(a.box), fromDecad(b.box)
		r.faceted = len(a.unread)+len(b.unread) > 0 || a.unmeshed || b.unmeshed
		s.matchFaces(&r, j, a, b, c)
	}
	if r.matched {
		return r, nil
	}
	r.port = explodeplan.Port(boxA, boxB)
	if j.Axis != nil && (j.Kind == PinInBore || j.Kind == PinInSlot) {
		ax := *j.Axis
		ax.Point = ax.Point.Add(ax.Dir.Scale(r.port.Sub(ax.Point).Dot(ax.Dir)))
		r.src, r.mode, r.dir, r.axis, r.port, r.portD = AxisDeclared, explodeplan.ModeAxis, ax.Dir, &ax, ax.Point, ax.Dir
		return r, nil
	}
	r.src, r.mode = BoxCentres, explodeplan.ModeCentres
	return r, nil
}

func (s *Set) matchFaces(r *reading, j Joint, a, b *bodyFaces, c config) {
	p := s.matchParams(c)
	switch j.Kind {
	case PinInBore:
		ms := jointinfer.MatchBore(a.cyls, b.cyls, p)
		if len(ms) == 0 {
			return
		}
		m := ms[0]
		for _, i := range m.Pin {
			r.facesA = append(r.facesA, a.cylFaces[i])
		}
		for _, i := range m.Bore {
			r.facesB = append(r.facesB, b.cylFaces[i])
		}
		r.setAxis(m.Axis, m.Length, m.Clearance, m.Bound)
	case PinInSlot:
		ms := jointinfer.MatchSlot(a.cyls, b.planes, p)
		if len(ms) == 0 {
			return
		}
		m := ms[0]
		r.facesA = []*decad.Face{a.cylFaces[m.Pin]}
		r.facesB = []*decad.Face{b.planeFaces[m.Walls[0]], b.planeFaces[m.Walls[1]]}
		r.setAxis(m.Axis, m.Length, m.Clearance, m.Bound)
	case FaceContact:
		ms := jointinfer.MatchFace(a.planes, b.planes, p)
		if len(ms) == 0 {
			return
		}
		m := ms[0]
		r.facesA, r.facesB = []*decad.Face{a.planeFaces[m.A]}, []*decad.Face{b.planeFaces[m.B]}
		r.src, r.mode, r.dir = FaceNormal, explodeplan.ModeAway, m.Normal.Scale(-1)
		r.port, r.portD, r.measured, r.bound, r.matched = m.Centre, m.Normal, m.Gap, a.planes[m.A].Bound+b.planes[m.B].Bound, true
	}
}

func (r *reading) setAxis(l jointinfer.Line, length, clearance, bound float64) {
	ax := Axis{Point: l.Point, Dir: l.Dir}
	r.src, r.mode, r.dir, r.axis = AxisFromFaces, explodeplan.ModeAxis, l.Dir, &ax
	r.port, r.portD, r.length = l.Point, l.Dir, length
	r.measured, r.bound, r.matched = clearance, bound, true
}

func toBox(b *decad.Body) explodeplan.Box {
	if b == nil {
		return explodeplan.Box{}
	}
	box, err := b.Bounds()
	if err != nil {
		return explodeplan.Box{}
	}
	return fromDecad(box)
}

func fromDecad(b decad.Box) explodeplan.Box {
	g := b.Bound.Base()
	grow := r3.Vec{X: g, Y: g, Z: g}
	return explodeplan.Box{Min: b.Min.Sub(grow), Max: b.Max.Add(grow)}
}

// Explode places every part along its joints for an exploded view. It reads the
// joints' axes from the faces and needs no Check. The joint graph is walked
// breadth-first from the ground, joints in Join order, so each part has one
// parent joint; a part reached twice keeps its first parent. Parts no joint
// reaches keep a zero offset and Depth -1. Each part moves along its parent
// joint's direction, in the sense whose swept box along the travel is strictly
// disjoint from every other part's box except those joined to it, else the
// shorter travel, then the sense with a positive largest component; the offset
// is the travel that clears the parent plus WithExplodeGap.
func (s *Set) Explode(ctx context.Context, opts ...ExplodeOption) (*Exploded, error) {
	c, err := foldOptions(opts)
	if err != nil {
		return nil, err
	}
	if ctx == nil || s == nil || s.doc == nil {
		return nil, fmt.Errorf("parts: %w: nil context or set", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := s.live(); err != nil {
		return nil, err
	}
	// Index the parts that have a body.
	var idx []int // plan index -> part index
	plan := map[int]int{}
	for i, p := range s.parts {
		if p.Body != nil {
			plan[i] = len(idx)
			idx = append(idx, i)
		}
	}
	if len(idx) == 0 {
		return nil, fmt.Errorf("parts: %w: no part has a body", decad.ErrDegenerate)
	}
	ground, err := s.pickGround(c.ground, idx, plan)
	if err != nil {
		return nil, err
	}
	fc := newFaceCache(false)
	boxes := make([]explodeplan.Box, len(idx))
	for k, i := range idx {
		bf, err := fc.get(ctx, s.parts[i].Body)
		if err != nil {
			return nil, err
		}
		boxes[k] = fromDecad(bf.box)
	}
	var joints []explodeplan.Joint
	var readings []reading
	var jointOf []int
	for ji, j := range s.joints {
		a, oka := plan[s.index[j.A]]
		b, okb := plan[s.index[j.B]]
		if !oka || !okb {
			continue
		}
		r, err := s.readJoint(ctx, j, fc, c)
		if err != nil {
			return nil, err
		}
		joints = append(joints, explodeplan.Joint{A: a, B: b, Mode: r.mode, Dir: r.dir, Source: int(r.src)})
		readings = append(readings, r)
		jointOf = append(jointOf, ji)
	}
	blk := &sweptBlocker{doc: s.doc, bodies: make([]*decad.Body, len(idx))}
	for k, i := range idx {
		blk.bodies[k] = s.parts[i].Body
	}
	res, err := explodeplan.Plan(ctx, boxes, joints, ground, c.gap, blk)
	if err != nil {
		return nil, err
	}
	return s.buildExploded(idx, ground, boxes, res, joints, readings, jointOf)
}

func (s *Set) buildExploded(idx []int, ground int, boxes []explodeplan.Box, res []explodeplan.Result,
	joints []explodeplan.Joint, readings []reading, jointOf []int) (*Exploded, error) {
	ex := &Exploded{Ground: s.parts[idx[ground]].Name}
	moved := make([]explodeplan.Box, len(boxes))
	for k, i := range idx {
		off := res[k].Offset
		t, err := r3.Translation(off)
		if err != nil {
			return nil, fmt.Errorf("parts: part %q: %w", s.parts[i].Name, err)
		}
		moved[k] = explodeplan.Box{Min: boxes[k].Min.Add(off), Max: boxes[k].Max.Add(off)}
		pull := PullNone
		if res[k].Joint >= 0 {
			pull = PullSource(res[k].Source)
		}
		ex.Parts = append(ex.Parts, ExplodedPart{Name: s.parts[i].Name, Offset: off, Placement: t,
			Depth: res[k].Depth, Anchor: moved[k].Centre(), Pull: pull})
	}
	for n, j := range joints {
		ex.Leaders = append(ex.Leaders, Leader{Joint: s.joints[jointOf[n]].Name,
			From: readings[n].port.Add(res[j.A].Offset), To: readings[n].port.Add(res[j.B].Offset)})
	}
	for a := range moved {
		for b := a + 1; b < len(moved); b++ {
			if explodeplan.Meet(moved[a], moved[b]) {
				ex.Overlaps = append(ex.Overlaps, [2]string{s.parts[idx[a]].Name, s.parts[idx[b]].Name})
			}
		}
	}
	return ex, nil
}

// pickGround resolves WithExplodeGround, defaulting to the manufactured part
// of largest volume.
func (s *Set) pickGround(name string, idx []int, plan map[int]int) (int, error) {
	if name != "" {
		i, ok := s.index[name]
		k, bodied := plan[i]
		if !ok || !bodied {
			return 0, fmt.Errorf("parts: %w: ground %q is unknown or has no body", decad.ErrDegenerate, name)
		}
		return k, nil
	}
	best, bestVol := -1, -1.0
	for k, i := range idx {
		if s.parts[i].Kind != Manufactured {
			continue
		}
		v, err := s.parts[i].Body.Volume()
		if err != nil {
			return 0, fmt.Errorf("parts: part %q: %w", s.parts[i].Name, err)
		}
		if vol := v.Value.Base(); vol > bestVol {
			best, bestVol = k, vol
		}
	}
	if best < 0 {
		return 0, fmt.Errorf("parts: %w: no manufactured part has a body", decad.ErrDegenerate)
	}
	return best, nil
}

// sweptBlocker answers explodeplan's path question with Document.SweptBox.
type sweptBlocker struct {
	doc    *decad.Document
	bodies []*decad.Body
}

func (b *sweptBlocker) at(ctx context.Context, part int, from, to r3.Vec) (decad.SweptBox, error) {
	f, err := r3.Translation(from)
	if err != nil {
		return decad.SweptBox{}, err
	}
	t, err := r3.Translation(to)
	if err != nil {
		return decad.SweptBox{}, err
	}
	return b.doc.SweptBox(ctx, b.bodies[part], decad.PoseSegment{From: f, To: t, Duration: units.Seconds(1)})
}

func (b *sweptBlocker) Clear(ctx context.Context, part int, from, to r3.Vec, offsets []r3.Vec, ignore []int) (bool, error) {
	moving, err := b.at(ctx, part, from, to)
	if err != nil {
		return false, err
	}
	for k := range b.bodies {
		if k == part || slices.Contains(ignore, k) {
			continue
		}
		still, err := b.at(ctx, k, offsets[k], offsets[k])
		if err != nil {
			return false, err
		}
		if !moving.StrictlyDisjoint(still) {
			return false, nil
		}
	}
	return true, nil
}
