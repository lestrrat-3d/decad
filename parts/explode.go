package parts

import (
	"context"
	"fmt"
	"math"
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
	PullNone         PullSource = iota // the ground, or a part no joint reaches
	AxisFromFaces                      // the axis of the matched cylinder faces
	AxisDeclared                       // the joint's declared Axis
	FaceNormal                         // the normal of the matched contact faces
	BoxCentres                         // the line between the two parts' box centres
	PullDeclared                       // the joint's declared Pull
	StageDirection                     // the direction its stage stated
	WorldAxis                          // derived: a world axis, or a direction another part's WithRemoval stated
	RemovalDirection                   // derived: the direction its own WithRemoval stated
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
	case PullDeclared:
		return "pull declared"
	case StageDirection:
		return "stage direction"
	case WorldAxis:
		return "world axis"
	case RemovalDirection:
		return "removal direction"
	}
	return "none"
}

// Exploded is the layout of a Set pulled apart along its joints.
type Exploded struct {
	Ground   string
	Parts    []ExplodedPart // in Set order, body-less parts omitted
	Leaders  []Leader
	Overlaps [][2]string // part pairs whose exploded boxes still meet
	Stages   []string    // the stage names in the order they ran; "" is the last, implicit stage
	Scale    units.Value // the WithExplodeScale multiplier, Dimensionless
	Short    []string    // parts whose Clearance is below their Gap, in Set order
	// Guides lists each moved part's path from its assembled place to its exploded place, in Parts
	// order (agent-docs/parts-design.md §5.10). decad draws nothing: the caller draws them as
	// polylines.
	Guides []Guide
	// Spread reports the layer spread of a derived explode (§5.9). It is nil without WithDerivedExplode
	// and when no derived part leaves along a world axis.
	Spread *Spread

	// Derived is true when WithDerivedExplode ran. The removal order of the parts no explicit stage
	// names was then read from tessellations, so the layout is layout-grade: a picture, not a proof
	// (agent-docs/parts-design.md §5.8). Its rounds run last-removed first, so the part removed last
	// sits nearest the assembly and Stages lists the rounds in that order. Blocked and Undecided are
	// empty without it.
	Derived bool
	// Blocked lists, in removal order, each pair that forced the order: a part that leaves along a
	// direction and the part that stood in the way along it, which therefore left earlier.
	Blocked []BlockedPair
	// Undecided lists the pairs whose verdict the tessellation could not settle and the layout read
	// as free: a part, a part still present when it leaves and the direction it leaves by. These are
	// the places where the layout is not certain.
	Undecided []UndecidedPair

	boxes []explodeplan.Box // the exploded boxes, in Parts order, for Hidden
}

// ExplodedPart is one part's place in the exploded layout.
type ExplodedPart struct {
	Name      string
	Offset    r3.Vec       // translation after the assembled pose
	Placement r3.Transform // the same, as a transform
	Depth     int          // joint steps from the ground; -1 when no joint reaches it
	Anchor    r3.Vec       // label anchor: exploded box centre
	Pull      PullSource
	Stage     string      // the last stage that moved it; "" for the ground and the implicit last stage
	Dir       r3.Vec      // the unit direction of that stage's move; zero when it did not move
	Travel    units.Value // the distance that stage's clearing rule needed, before the gap and scale
	Gap       units.Value // the gap asked for there: its WithPartGap, else its stage's, else WithExplodeGap
	Moved     units.Value // the length of that move, scale included
	Clearance units.Value // after the last stage and the scale: the least box distance to any part it is not joined to
	Path      PathState
	Blocker   string // the part that blocked or enclosed the path, or that Clearance was read against
	// Order is the 1-based position in the derived removal order, and Round the 1-based round the
	// part left in. Both are 0 for the ground, for a part an explicit stage moved, and without
	// WithDerivedExplode. For a derived part Pull is AxisFromFaces, AxisDeclared, PullDeclared,
	// WorldAxis or RemovalDirection.
	Order, Round int
	// Layer is the part's layer in the spread (agent-docs/parts-design.md §5.9): +k or -k for the
	// k-th derived round from the base leaving along +Spread.Axis or -Spread.Axis, 0 for the base. It
	// is 0 when Spread is nil.
	Layer int
	// Slide is what the spread added to Offset: the push along Spread.Axis and the move to the
	// layer's cell, after WithExplodeScale. It is zero when the spread did not run. Moved does not
	// include it.
	Slide r3.Vec
}

// Guide is the path one part moved along, for a dotted guide line from its exploded place back to
// where it assembles. Points runs from the centre of the part's assembled box through the centre
// after each move (a carry with its parent, each stage's move, the spread's push along its axis and
// its move to its cell) to its Anchor, in millimetres and after WithExplodeScale. Consecutive points
// differ, and there are at least two. A guide may pass through other parts' boxes.
type Guide struct {
	Part   string
	Points []r3.Vec
}

// BlockedPair is one pair that forced the derived removal order: Blocker stood ahead of Mover
// along Dir, so Blocker left first. Point is a point on Mover's surface under Blocker, in
// millimetres, and Height how far Blocker's surface stands ahead of it along Dir.
type BlockedPair struct {
	Mover, Blocker string
	Dir            r3.Vec
	Point          r3.Vec
	Height         units.Value
}

// UndecidedPair is a pair the tessellation could not settle: Other may stand ahead of Mover along
// Dir only inside the mesh bounds' uncertainty, and the layout read it as free.
type UndecidedPair struct {
	Mover, Other string
	Dir          r3.Vec
}

// Leader is the dotted line one joint draws in the exploded view.
type Leader struct {
	Joint    string
	From, To r3.Vec // exploded coordinates: the port on A, the port on B
	// Separation is the distance between the two parts' exploded boxes, zero
	// when they meet.
	Separation units.Value
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
//
// WithExplodeStages runs that in ordered stages, WithPartGap and
// WithExplodeScale tune the distances, and Joint.Pull states a direction
// (agent-docs/parts-design.md §5.3). WithDerivedExplode derives the order and
// direction of the parts no stage names from their tessellations (§5.8); for a
// thin assembly whose layers are not coaxial it also spreads the layers across
// their axis (§5.9). Guides records each moved part's path (§5.10).
func (s *Set) Explode(ctx context.Context, opts ...ExplodeOption) (*Exploded, error) {
	c, err := foldOptions(opts)
	if err != nil {
		return nil, err
	}
	if len(c.removals) > 0 && !c.derived {
		return nil, fmt.Errorf("parts: %w: WithRemoval needs WithDerivedExplode", decad.ErrDegenerate)
	}
	if c.noSpread && !c.derived {
		return nil, fmt.Errorf("parts: %w: WithoutExplodeSpread needs WithDerivedExplode", decad.ErrDegenerate)
	}
	if ctx == nil || s == nil || s.doc == nil {
		return nil, fmt.Errorf("parts: %w: nil context or set", decad.ErrDegenerate)
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
	stages, err := s.resolveStages(c, ground, idx, plan)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
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
	var jointAxes []jointAxis // the axes and pulls the derived search tries first
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
		if err := applyPull(&r, j, c); err != nil {
			return nil, err
		}
		if r.axis != nil {
			jointAxes = append(jointAxes, jointAxis{dir: r.axis.Dir, src: r.src})
		}
		if _, ok := j.Pull.Normalize(); ok {
			jointAxes = append(jointAxes, jointAxis{dir: j.Pull, src: PullDeclared})
		}
		joints = append(joints, explodeplan.Joint{A: a, B: b, Mode: r.mode, Dir: r.dir, Source: int(r.src)})
		readings = append(readings, r)
		jointOf = append(jointOf, ji)
	}
	blk := &sweptBlocker{doc: s.doc, bodies: make([]*decad.Body, len(idx))}
	for k, i := range idx {
		blk.bodies[k] = s.parts[i].Body
	}
	order, nodes := explodeplan.Walk(len(idx), joints, ground)
	runNodes := nodes
	var rep *derivedReport
	if c.derived {
		d, err := s.deriveStages(ctx, c, derivedInput{ground: ground, idx: idx, plan: plan, boxes: boxes, stages: stages,
			walk: order, nodes: nodes, faces: fc, jointAxes: jointAxes})
		if err != nil {
			return nil, err
		}
		stages.stages, stages.names, order, runNodes, rep = d.stages, d.names, d.order, d.nodes, &d.report
	}
	var spreadReq *spreadPlan
	if rep != nil {
		spreadReq = planSpread(c, boxes, rep)
	}
	in := explodeplan.Input{Boxes: boxes, Joints: joints, Ground: ground, Order: order, Nodes: runNodes,
		Stages: stages.stages, DefaultGap: c.gap, PartGap: stages.partGap, Scale: c.scale, Blocker: blk}
	if spreadReq != nil {
		in.Spread = &spreadReq.req
	}
	out, err := explodeplan.Run(ctx, in)
	if err != nil {
		return nil, err
	}
	ex, err := s.buildExploded(idx, ground, boxes, out.Parts, nodes, joints, readings, jointOf, stages.names, c.scale, rep)
	if err != nil {
		return nil, err
	}
	if spreadReq != nil {
		ex.Spread = spreadReq.report(out.Spread)
		for k := range ex.Parts {
			ex.Parts[k].Layer = spreadReq.req.Layer[k]
		}
	}
	return ex, nil
}

// applyPull folds a joint's declared Pull into its reading: it replaces the face
// normal and the box-centre line, and on an axis it picks the sense. A Pull at
// right angles to the axis names no sense and is refused.
func applyPull(r *reading, j Joint, c config) error {
	u, ok := j.Pull.Normalize()
	if !ok {
		return nil
	}
	if r.mode == explodeplan.ModeAxis && r.axis != nil {
		along := r.dir.Dot(u)
		if math.Abs(along) <= math.Sin(c.axisTol) {
			return fmt.Errorf("parts: %w: the pull of joint %q is at right angles to its axis", decad.ErrDegenerate, j.Name)
		}
		r.dir = r.dir.Scale(math.Copysign(1, along))
	} else {
		r.dir = u
	}
	r.src, r.mode = PullDeclared, explodeplan.ModeAway
	return nil
}

func (s *Set) buildExploded(idx []int, ground int, boxes []explodeplan.Box, res []explodeplan.Result, nodes []explodeplan.Node,
	joints []explodeplan.Joint, readings []reading, jointOf []int, stageNames []string, scale float64, rep *derivedReport) (*Exploded, error) {
	ex := &Exploded{Ground: s.parts[idx[ground]].Name, Stages: stageNames, Scale: units.Scalar(scale)}
	if rep != nil {
		ex.Derived, ex.Blocked, ex.Undecided = true, rep.blocked, rep.undecided
	}
	moved := make([]explodeplan.Box, len(boxes))
	mm := units.Millimeters
	for k, i := range idx {
		off := res[k].Offset
		t, err := r3.Translation(off)
		if err != nil {
			return nil, fmt.Errorf("parts: part %q: %w", s.parts[i].Name, err)
		}
		moved[k] = explodeplan.Box{Min: boxes[k].Min.Add(off), Max: boxes[k].Max.Add(off)}
		pull := PullNone
		switch {
		case res[k].Source == explodeplan.StageDirection:
			pull = StageDirection
		case nodes[k].Joint >= 0:
			pull = PullSource(res[k].Source)
		}
		order, round := 0, 0
		if rep != nil && rep.order[k] > 0 {
			pull, order, round = rep.src[k], rep.order[k], rep.round[k]
		}
		stage := ""
		if res[k].Stage >= 0 {
			stage = stageNames[res[k].Stage]
		}
		blocker := ""
		if res[k].Blocker >= 0 {
			blocker = s.parts[idx[res[k].Blocker]].Name
		}
		ex.Parts = append(ex.Parts, ExplodedPart{Name: s.parts[i].Name, Offset: off, Placement: t,
			Depth: nodes[k].Depth, Anchor: moved[k].Centre(), Pull: pull, Stage: stage, Dir: res[k].Dir,
			Travel: mm(res[k].Travel), Gap: mm(res[k].Gap), Moved: mm(res[k].Moved), Clearance: mm(res[k].Clearance),
			Path: PathState(res[k].Path), Blocker: blocker, Order: order, Round: round, Slide: res[k].Slide})
		if pts := guidePoints(boxes[k], res[k].Stops); len(pts) >= 2 {
			ex.Guides = append(ex.Guides, Guide{Part: s.parts[i].Name, Points: pts})
		}
		if res[k].Short {
			ex.Short = append(ex.Short, s.parts[i].Name)
		}
	}
	for n, j := range joints {
		ex.Leaders = append(ex.Leaders, Leader{Joint: s.joints[jointOf[n]].Name,
			From: readings[n].port.Add(res[j.A].Offset), To: readings[n].port.Add(res[j.B].Offset),
			Separation: mm(explodeplan.BoxDistance(moved[j.A], moved[j.B]))})
	}
	for a := range moved {
		for b := a + 1; b < len(moved); b++ {
			if explodeplan.Meet(moved[a], moved[b]) {
				ex.Overlaps = append(ex.Overlaps, [2]string{s.parts[idx[a]].Name, s.parts[idx[b]].Name})
			}
		}
	}
	ex.boxes = moved
	return ex, nil
}

// guidePoints is the centre of box moved by each stop, computed as ExplodedPart.Anchor is.
func guidePoints(box explodeplan.Box, stops []r3.Vec) []r3.Vec {
	out := make([]r3.Vec, len(stops))
	for i, st := range stops {
		out[i] = explodeplan.Box{Min: box.Min.Add(st), Max: box.Max.Add(st)}.Centre()
	}
	return out
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

func (b *sweptBlocker) Path(ctx context.Context, part int, from, to r3.Vec, offsets []r3.Vec, ignore []int) (explodeplan.PathState,
	int, error) {
	moving, err := b.at(ctx, part, from, to)
	if err != nil {
		return 0, -1, err
	}
	start, err := b.at(ctx, part, from, from)
	if err != nil {
		return 0, -1, err
	}
	enclosed, blocked := -1, -1
	for k := range b.bodies {
		if k == part || slices.Contains(ignore, k) {
			continue
		}
		still, err := b.at(ctx, k, offsets[k], offsets[k])
		if err != nil {
			return 0, -1, err
		}
		if moving.StrictlyDisjoint(still) {
			continue
		}
		if !start.StrictlyDisjoint(still) {
			if enclosed < 0 {
				enclosed = k
			}
		} else if blocked < 0 {
			blocked = k
		}
	}
	switch {
	case enclosed >= 0:
		return explodeplan.PathEnclosed, enclosed, nil
	case blocked >= 0:
		return explodeplan.PathBlocked, blocked, nil
	}
	return explodeplan.PathClear, -1, nil
}
