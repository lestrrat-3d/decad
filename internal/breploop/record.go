package breploop

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// State owns one route-L record reading and its shared work budget.
type State struct {
	Payload      breprecord.Payload
	Topology     *breprecord.Topology
	Budget       *proofbound.WorkBudget
	LoopUse      map[[3]int]int
	Matched      []int
	Op           string
	Kind         breprecord.BandKind
	Setback      *capblend.Setback
	Selector     string
	SelectedText string
}

// Sel is one touched planar loop and its classified neighbouring faces.
type Sel struct {
	Face, Loop int
	Beside     []Beside
	Sigma      float64
	Setback    capblend.Setback
}

// Beside is the use and far levels of one face beside a touched loop.
type Beside struct {
	Use    int
	Planar bool
	Far    []float64
	Nb     [2]int
}

// New builds one topology and budget, in the existing admission order.
func New(ctx context.Context, bp breprecord.Payload, op string, kind breprecord.BandKind,
	setback *capblend.Setback) (*State, error) {
	topo, err := breprecord.TopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	r := &State{Payload: bp, Topology: topo, Budget: proofbound.NewWorkBudget(ctx),
		LoopUse: map[[3]int]int{}, Op: op, Kind: kind, Setback: setback}
	for ui, u := range topo.Uses {
		if u.Part == brepgeom.LoopSeg {
			r.LoopUse[[3]int{u.Face, u.Loop, u.Seg}] = ui
		}
	}
	return r, nil
}

// Refuse names a route-L refusal and its original complete selection.
func (r *State) Refuse(row, reason string) error {
	return r.RefuseWithSelection(row, reason, r.SelectedText)
}

// RefuseWithSelection names only an ordered subset of the original selection.
func (r *State) RefuseWithSelection(row, reason, selectedText string) error {
	return fmt.Errorf(`%w: this evaluator %s complete loops of planar brep faces through route L only where Table LB admits them; %s (modify-general %s); selector %s matched [%s]`,
		decaderr.ErrUnsupported, r.Op, reason, row, r.Selector, selectedText)
}

// partner is the other use of use ui's edge.
func (r *State) partner(ui int) int {
	pair := r.Topology.Edges[r.Topology.EdgeOf[ui]]
	if pair[0] == ui {
		return pair[1]
	}
	return pair[0]
}

// naturalFaceLoops is LB4 over one face: every segment runs over its natural
// range, and no loop of a planar face holds two consecutive segments on one
// carrier (route E's EB5). It returns the reason a face fails, or "".
func (r *State) naturalFaceLoops(fi int) (string, error) {
	f := r.Payload.Faces[fi]
	if !f.Planar() {
		if !brepgeom.NaturalRange(f.Wall) {
			return fmt.Sprintf(`brep face %s's wall runs over a narrowed range`, f.Role), nil
		}
		return "", nil
	}
	loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *f.Region)
	if err != nil {
		return "", err
	}
	for li, loop := range append([]sectionrecord.LoopRecord{f.Region.Outer}, f.Region.Holes...) {
		for _, seg := range loop.Segments {
			if !brepgeom.NaturalRange(seg) {
				return fmt.Sprintf(`brep face %s holds a segment over a narrowed range`, f.Role), nil
			}
		}
		walks := loops[li].Walks
		if len(walks) != len(loop.Segments) {
			return fmt.Sprintf(`brep face %s holds two consecutive segments on one line`, f.Role), nil
		}
		n := len(walks)
		for i, w := range walks {
			next := walks[(i+1)%n]
			if n > 1 && w.IsCircular() && next.IsCircular() && !w.Closed && !next.Closed &&
				w.CU == next.CU && w.CV == next.CV && w.Radius == next.Radius && (w.Th1 > w.Th0) == (next.Th1 > next.Th0) {
				return fmt.Sprintf(`brep face %s holds two consecutive arcs on one circle`, f.Role), nil
			}
		}
	}
	return "", nil
}

// classifyLoop is LB3, LB4 and LB6 (SL2) on one touched loop of face F,
// whose normal runs along reference axis n. Each segment's edge pairs with a
// face that is (sw), swept along n with the segment as a rim over its natural
// range and no side-line split, or (pl), planar across another axis, holding
// the segment as a loop segment whose two neighbours are lines along n. Every
// such face, and F, passes LB4. The faces' far ends must all lie on one side
// of F (LB6): against F's outward normal the walls descend into the body
// (sigma −1), along it they rise off F (sigma +1).
func (r *State) ClassifyLoop(sel *Sel) error {
	f := r.Payload.Faces[sel.Face]
	eF := r.Topology.Embeds[sel.Face]
	n := eF.Axis[2]
	level := eF.Canon(0, 0, f.Z0)[n]
	outward := -eF.Sign[2]
	if f.Outward {
		outward = eF.Sign[2]
	}
	refuse := func(reason string) error { return r.Refuse("SL2", reason) }
	why, err := r.naturalFaceLoops(sel.Face)
	if err != nil {
		return err
	}
	if why != "" {
		return refuse(why)
	}
	segs := len(r.Topology.Planar[sel.Face][sel.Loop])
	sel.Beside = make([]Beside, segs)
	for s := range segs {
		if err := survey2d.WallBudgetStep(r.Budget); err != nil {
			return err
		}
		pu := r.Topology.Uses[r.partner(r.LoopUse[[3]int{sel.Face, sel.Loop, s}])]
		a := r.Payload.Faces[pu.Face]
		eA := r.Topology.Embeds[pu.Face]
		b := Beside{Use: r.partner(r.LoopUse[[3]int{sel.Face, sel.Loop, s}])}
		switch {
		case !a.Planar() && eA.Axis[2] == n && brepgeom.IsRim(pu.Part):
			if len(a.Side0) > 0 || len(a.Side1) > 0 {
				return refuse(fmt.Sprintf(`brep face %s beside the loop has a split side line`, a.Role))
			}
			far := a.Z1
			if pu.Part == brepgeom.Rim1 {
				far = a.Z0
			}
			b.Far = []float64{eA.Sign[2]*far + 0}
		case a.Planar() && eA.Axis[2] != n && pu.Part == brepgeom.LoopSeg:
			b.Planar = true
			nA := len(r.Topology.Planar[pu.Face][pu.Loop])
			for k, s2 := range [2]int{(pu.Seg + nA - 1) % nA, (pu.Seg + 1) % nA} {
				ui := r.LoopUse[[3]int{pu.Face, pu.Loop, s2}]
				nu := r.Topology.Uses[ui]
				far, ok := brepLineAlong(nu, n, level)
				if !ok {
					return refuse(fmt.Sprintf(`brep face %s beside the loop continues past the loop's vertex on a segment that is not a straight line along the face's normal`, a.Role))
				}
				b.Far = append(b.Far, far)
				b.Nb[k] = ui
			}
		default:
			return refuse(fmt.Sprintf(`brep face %s beside the loop neither sweeps along brep face %s's normal nor holds the loop's edge as a plane across another axis`, a.Role, f.Role))
		}
		why, err := r.naturalFaceLoops(pu.Face)
		if err != nil {
			return err
		}
		if why != "" {
			return refuse(why)
		}
		for _, far := range b.Far {
			sense := -1.0
			if (far > level) == (outward > 0) {
				sense = 1
			}
			if sel.Sigma == 0 {
				sel.Sigma = sense
			}
			if sel.Sigma != sense {
				return refuse(fmt.Sprintf(`the faces beside brep face %s's loop leave it on both sides`, f.Role))
			}
		}
		sel.Beside[s] = b
	}
	return nil
}

// brepLineAlong reads one neighbouring segment of a (pl) face: a straight
// line whose ends differ only in reference coordinate n, one end at level.
// It returns the other end's coordinate along n.
func brepLineAlong(u brepgeom.Use, n int, level float64) (float64, bool) {
	if !u.Walk.IsLine() {
		return 0, false
	}
	for i := range 3 {
		if i != n && u.From[i] != u.To[i] {
			return 0, false
		}
	}
	switch {
	case u.From[n] == level && u.To[n] != level:
		return u.To[n], true
	case u.To[n] == level && u.From[n] != level:
		return u.From[n], true
	default:
		return 0, false
	}
}

func (r *State) RefuseAsymmetricLoop(row, reason string) error {
	return fmt.Errorf(`%w: %s (modify-reach %s); selector %s matched [%s]`, decaderr.ErrUnsupported,
		reason, row, r.Selector, r.SelectedText)
}

// requireBandReach is LB5 (reach SX7): the band reaches ds along every face
// beside its loop and must stop short of that face's far end. The reach is
// summed per face end: a (sw) face trimmed from both ends, or a (pl) face's
// neighbouring line claimed from both, must keep a part of its own.
func (r *State) RequireBandReach(sels []Sel) error {
	type piece struct{ face, use int }
	reach := map[piece]float64{}
	for _, sel := range sels {
		f := r.Payload.Faces[sel.Face]
		eF := r.Topology.Embeds[sel.Face]
		level := eF.Canon(0, 0, f.Z0)[eF.Axis[2]]
		for _, b := range sel.Beside {
			a := r.Payload.Faces[r.Topology.Uses[b.Use].Face]
			for k, far := range b.Far {
				key := piece{face: r.Topology.Uses[b.Use].Face, use: -1}
				height := math.Abs(far - level)
				if b.Planar {
					key.use = b.Nb[k]
				}
				reach[key] += sel.Setback.Ds
				if reach[key] >= height {
					return fmt.Errorf(`%w: the %s band on brep face %s's loop reaches or passes the far end of brep face %s beside it (%g mm of %g mm); a merging kernel is not available (modify-reach SX7); selector %s matched [%s]`,
						decaderr.ErrUnsupported, r.Kind, f.Role, a.Role, reach[key], height, r.Selector, r.SelectedText)
				}
			}
		}
	}
	return nil
}

// loopFaceView is the cap blend view of one face F and its touched loops:
// F's region and level, each loop chamfered on the cap its band's material
// sense names, both caps at the call's setbacks. The shipped cap-loop gates
// read F's loops through it exactly as they read a prism's cap loops.
func (r *State) loopFaceView(fi int, sels []Sel) capblend.Payload {
	f := r.Payload.Faces[fi]
	cbp := capblend.Payload{
		Profile: *f.Region, Frame: f.Frame, Xform: r.Payload.Xform,
		Z0: f.Z0, Z1: f.Z0, Z0Delta: f.Z0Delta, Z1Delta: f.Z0Delta,
		Start: *r.Setback, End: *r.Setback,
		StartLoops: map[int]bool{}, EndLoops: map[int]bool{},
		Fillet: r.Kind == breprecord.BandFillet,
	}
	for _, sel := range sels {
		if sel.Face != fi {
			continue
		}
		band := breprecord.LoopBand{Sigma: sel.Sigma, Setback: sel.Setback}
		if band.MatSign(f) > 0 {
			cbp.StartLoops[sel.Loop] = true
			cbp.Start = sel.Setback
		} else {
			cbp.EndLoops[sel.Loop] = true
			cbp.End = sel.Setback
		}
	}
	return cbp
}

// rewriteLoopFaces is modify-general §4.2 steps 1 to 5 over every touched
// loop, after Table LB's topology gates (stage 3: LB3, LB4, LB6, then LB5). Per
// face F holding touched loops, stage 4 builds each cap contour (SX6) and
// decides SX13's axial and radial halves, and a fillet's corners take Table
// LF's classes (loop-fillet SF1); stage 5 runs SX14, then F's
// rewritten region faces modify §5's audit (S8, S7 — the contour against F's
// other loops, read as SX7/SX12 — and S9) at the setback and at the top of its
// conversion span. Each (sw) face's level at F moves to the side level, and
// each (pl) face's segment on the loop moves there along F's normal with its
// two neighbouring lines shortened by d, the connector-free trim route E
// applies, and the (pl) face then faces the same audit (S8, S6, S7, S9). F's
// section displacement takes every touched loop's contour displacement
// (modify-general §7). The result keeps the receiver's bands and records one
// more per touched loop.
func (r *State) RewriteLoopFaces(ctx context.Context, sels []Sel) (breprecord.Payload, error) {
	var faceOrder []int
	for _, sel := range sels {
		if !slices.Contains(faceOrder, sel.Face) {
			faceOrder = append(faceOrder, sel.Face)
		}
	}

	// Stage 4: the contours exist (SX6), and the setback survives float64 at
	// each side level (SX13's axial half) and each circular wall's radius
	// (its radial half).
	mixed := make(map[int]momentinput.Profile, len(faceOrder))
	views := make(map[int]capblend.Payload, len(faceOrder))
	for _, fi := range faceOrder {
		cbp := r.loopFaceView(fi, sels)
		views[fi] = cbp
		profile, err := capblend.MixedOffsetProfile(r.Budget, cbp)
		if err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, capband.ChamferDropError(err))
		}
		mixed[fi] = profile
		if err := capblend.RequireLevelsSeparate(cbp); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, err)
		}
	}
	work := freeform.NewFreeformWork()
	for _, sel := range sels {
		setback := sel.Setback
		cl, err := capblend.OneLoopCornerLoop(r.Budget, r.Payload.Faces[sel.Face].RegionLoop(sel.Loop), work)
		if err != nil {
			return breprecord.Payload{}, err
		}
		if len(cl.Walks) == 1 && cl.Walks[0].Closed {
			_, err = capband.BandRadius(cl.Walks[0], setback.Dc, offset2d.ShellTol)
		} else {
			_, err = views[sel.Face].OffsetJoins(r.Budget, sel.Loop, cl, setback.Dc)
		}
		if err != nil {
			return breprecord.Payload{}, r.WrapFace(sel.Face, err)
		}
		// SF1 (loop-fillet §6): every corner of a fillet's loop takes a class of
		// Table LF.
		if r.Kind == breprecord.BandFillet {
			f := r.Payload.Faces[sel.Face]
			if _, err := breprecord.FilletLoopOf(r.Budget, f.RegionLoop(sel.Loop), setback.Dc, f.Role, work); err != nil {
				return breprecord.Payload{}, r.WrapFace(sel.Face, err)
			}
		}
	}

	// Stage 5: SX14 at every miter corner beside a circular wall, then each
	// face's rewritten region.
	for _, fi := range faceOrder {
		if err := capblend.RequireCornerLoci(r.Budget, views[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, err)
		}
		region := *r.Payload.Faces[fi].Region
		if err := offset2d.AuditOffsetSectionBudget(r.Budget, region, mixed[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, capband.ChamferAuditError(err))
		}
		if err := capblend.AuditSetbackSpan(r.Budget, region, views[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, err)
		}
	}

	faces := slices.Clone(r.Payload.Faces)
	for _, fi := range faceOrder {
		region := mixed[fi]
		faces[fi].Region = &region
	}
	trims := map[int][]map[int]*offset2d.Blend{}
	bands := slices.Clone(r.Payload.LoopBands)
	for _, sel := range sels {
		setback := sel.Setback
		f := r.Payload.Faces[sel.Face]
		eF := r.Topology.Embeds[sel.Face]
		n := eF.Axis[2]
		band := breprecord.LoopBand{Face: sel.Face, Loop: sel.Loop, Orig: sectionrecord.CloneLoopRecord(f.RegionLoop(sel.Loop)), Setback: setback, Sigma: sel.Sigma, Kind: r.Kind}
		delta, err := views[sel.Face].LoopContourDelta(ctx, sel.Loop, band.Orig, setback.Dc, setback.DcDelta)
		if err != nil {
			return breprecord.Payload{}, r.WrapFace(sel.Face, err)
		}
		faces[sel.Face].Delta = math.Max(faces[sel.Face].Delta, delta)
		sideZ, sideDelta := band.SideLevel(f)
		sideRef := eF.Canon(0, 0, sideZ)[n]
		for _, b := range sel.Beside {
			pu := r.Topology.Uses[b.Use]
			eA := r.Topology.Embeds[pu.Face]
			a := &faces[pu.Face]
			if !b.Planar {
				local := eA.Sign[2]*sideRef + 0
				if pu.Part == brepgeom.Rim0 {
					a.Z0, a.Z0Delta = local, proofbound.AbsSumUpper(math.Max(a.Z0Delta, f.Z0Delta), sideDelta)
				} else {
					a.Z1, a.Z1Delta = local, proofbound.AbsSumUpper(math.Max(a.Z1Delta, f.Z0Delta), sideDelta)
				}
				continue
			}
			if err := r.TrimPlanar(trims, pu, n, sideRef, setback.Ds); err != nil {
				return breprecord.Payload{}, err
			}
			a.Delta = math.Max(a.Delta, sideDelta)
		}
		bands = append(bands, band)
	}

	// The (pl) faces' audit, after every face F's.
	var planarOrder []int
	for fi := range trims {
		planarOrder = append(planarOrder, fi)
	}
	slices.Sort(planarOrder)
	for _, fi := range planarOrder {
		f := r.Payload.Faces[fi]
		loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *f.Region)
		if err != nil {
			return breprecord.Payload{}, err
		}
		profile, _, err := offset2d.RewriteProfileBudget(r.Budget, *f.Region, loops, trims[fi])
		if err != nil {
			return breprecord.Payload{}, err
		}
		if err := offset2d.AuditRewriteBudget(r.Budget, *f.Region, profile, loops, trims[fi]); err != nil {
			return breprecord.Payload{}, r.WrapFace(fi, sectionaudit.RenderCoordinates(err))
		}
		faces[fi].Region = &profile
	}
	out := breprecord.Payload{Faces: faces, Xform: r.Payload.Xform, LoopBands: bands}
	out.AssignRoles()
	return out, nil
}

// trimPlanar marks one (pl) face's trim: the segment use pu walks moves to
// the side level along reference axis n, as a connector-free blend at each
// of its two corners whose foot is the moved end and whose cutback on the
// neighbouring line is d. A corner two bands claim is SL2.
func (r *State) TrimPlanar(trims map[int][]map[int]*offset2d.Blend, pu brepgeom.Use, n int, sideRef, d float64) error {
	f := r.Payload.Faces[pu.Face]
	loops, err := offset2d.ProfileCornerLoopsBudget(r.Budget, *f.Region)
	if err != nil {
		return err
	}
	if trims[pu.Face] == nil {
		trims[pu.Face] = make([]map[int]*offset2d.Blend, len(loops))
		for i := range trims[pu.Face] {
			trims[pu.Face][i] = map[int]*offset2d.Blend{}
		}
	}
	e := r.Topology.Embeds[pu.Face]
	walks := loops[pu.Loop].Walks
	count := len(walks)
	seg := walks[pu.Seg]
	for c, at := range [2][2]float64{{seg.StartU, seg.StartV}, {seg.EndU, seg.EndV}} {
		moved := e.Canon(at[0], at[1], f.Z0)
		moved[n] = sideRef
		local := e.Local(moved)
		p := sectionrecord.Point2{U: local[0], V: local[1]}
		trim := &offset2d.Blend{FA: p, FB: p}
		ci := pu.Seg
		if c == 0 {
			trim.CutbackA = d
		} else {
			trim.CutbackB = d
			ci = (pu.Seg + 1) % count
		}
		if trims[pu.Face][pu.Loop][ci] != nil {
			return r.Refuse("SL2", fmt.Sprintf(`a corner of brep face %s beside the loop is claimed by two bands`, f.Role))
		}
		trims[pu.Face][pu.Loop][ci] = trim
	}
	return nil
}

// wrapFace names the face a refusal of one face's rewrite came from and the
// selection, passing a context error through unwrapped.
func (r *State) WrapFace(fi int, err error) error {
	if errors.Is(err, context.Canceled) {
		return context.Canceled
	}
	if errors.Is(err, context.DeadlineExceeded) {
		return context.DeadlineExceeded
	}
	return fmt.Errorf(`%w; in brep face %s; selector %s matched [%s]`,
		err, r.Payload.Faces[fi].Role, r.Selector, r.SelectedText)
}
