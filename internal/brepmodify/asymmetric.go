package brepmodify

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/modifyoption"
	"github.com/lestrrat-3d/decad/internal/modifyselect"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Asymmetric is a resolved WithAsymmetricChamfer: the receiver whose
// face roles map a reference face to a walk of the recorded section, each
// chamfered edge's reference face, and the two distances in millimetres —
// d, the positional distance, across the reference face, and other across
// the face beside it — each beside the rounding its own unit conversion
// committed.
type Asymmetric struct {
	Body               *topo.Body
	Refs               map[*topo.Edge]*topo.Face
	RecordRoles        []string
	D, Other           float64
	DDelta, OtherDelta float64
}

// Setbacks returns the arc lengths a chamfer of corner ci of loop li sets
// back along the arriving and the leaving walk (docs/modify-reach-design.md
// §6): the walk whose wall is e's reference face takes d and the other walk
// takes other. A side wall's side(i,j) roles name the recorded segments it
// is built from, and the reference face's roles decide which walk it is; a
// face whose roles name segments of both walks or of neither is
// decaderr.ErrUnsupported, since this evaluator cannot tell which setback it takes.
func (a *Asymmetric) Setbacks(loop survey2d.CornerLoop, li, ci int, e *topo.Edge) (float64, float64, error) {
	ref := a.Refs[e]
	if ref == nil {
		return 0, 0, fmt.Errorf(`%w: the asymmetric chamfer has no reference face for this edge`, decaderr.ErrUnsupported)
	}
	segs := faceSideSegments(a.Body, ref, li)
	n := len(loop.Walks)
	arriving := modifyoption.WalkHoldsAny(loop.Walks[(ci+n-1)%n].Segs, segs)
	leaving := modifyoption.WalkHoldsAny(loop.Walks[ci].Segs, segs)
	switch {
	case arriving && !leaving:
		return a.D, a.Other, nil
	case leaving && !arriving:
		return a.Other, a.D, nil
	default:
		return 0, 0, fmt.Errorf(`%w: the asymmetric chamfer's reference face is not the wall of exactly one of the two walks meeting at loop %d corner %d`, decaderr.ErrUnsupported, li, ci)
	}
}

// CapSetbacks returns the start and the end cap's own two setbacks for a
// two-distance chamfer of complete cap loops (docs/modify-reach-design.md
// §8.3.1). Every edge of a cap loop borders its cap face and one side wall, and
// its reference face picks between them: the cap face gives the cap dc = d and
// ds = other, a side wall gives dc = other and ds = d. The pick is made per
// cap. A cap whose edges pick differently, and a loop chamfered on both caps
// whose caps pick differently, give one loop's patches two in-plane offsets,
// and both are SX4 (decaderr.ErrUnsupported). SX3 has already refused both shapes — a
// cap face borders every edge of its loops, and a side wall borders that
// loop's edges on both caps, so a mixed pick names some edge's two faces or
// neither — and these arms are a second check over the resolved pairs. An
// edge on neither cap face is decaderr.ErrUnsupported; classifyChamferSelection has
// already put every selected edge on one. A cap with no selected edge reads
// the side-wall pick, which no band reads.
func (a *Asymmetric) CapSetbacks(caps PrismCaps, edges []*topo.Edge, startLoops, endLoops map[int]bool) (capblend.Setback, capblend.Setback, error) {
	var seen, capRef [2]bool
	for ei, e := range edges {
		c, capFace := -1, (*topo.Face)(nil)
		for _, f := range e.Faces {
			switch {
			case f == caps.Start && f != nil:
				c, capFace = 0, f
			case f == caps.End && f != nil:
				c, capFace = 1, f
			}
		}
		if c < 0 {
			return capblend.Setback{}, capblend.Setback{}, fmt.Errorf(`%w: the asymmetric chamfer's %s borders neither cap face`, decaderr.ErrUnsupported, modifyselect.EdgeContext(ei, e))
		}
		ref := a.Refs[e] == capFace
		if seen[c] && capRef[c] != ref {
			return capblend.Setback{}, capblend.Setback{}, fmt.Errorf(`%w: the asymmetric chamfer's reference names the cap face for some edges of one cap and a side wall for others; one cap takes one assignment (modify-reach SX4)`, decaderr.ErrUnsupported)
		}
		seen[c], capRef[c] = true, ref
	}
	if seen[0] && seen[1] && capRef[0] != capRef[1] {
		for li, on := range startLoops {
			if on && endLoops[li] {
				return capblend.Setback{}, capblend.Setback{}, fmt.Errorf(`%w: the asymmetric chamfer's reference names the cap face on one cap of loop %d and its side walls on the other; one loop takes one assignment (modify-reach SX4)`, decaderr.ErrUnsupported, li)
			}
		}
	}
	pick := func(capReferenced bool) capblend.Setback {
		if capReferenced {
			return capblend.Setback{Dc: a.D, DcDelta: a.DDelta, Ds: a.Other, DsDelta: a.OtherDelta}
		}
		return capblend.Setback{Dc: a.Other, DcDelta: a.OtherDelta, Ds: a.D, DsDelta: a.DDelta}
	}
	return pick(capRef[0]), pick(capRef[1]), nil
}

// faceSideSegments returns the recorded segments of loop li that face f is
// built from, read from its side(i,j) roles under b's own producer.
func faceSideSegments(b *topo.Body, f *topo.Face, li int) map[int]struct{} {
	var roles []string
	for _, o := range f.Origins {
		if o.Producer == b.Origin.Producer {
			roles = append(roles, o.Role)
		}
	}
	return modifyoption.SideSegments(roles, li)
}
