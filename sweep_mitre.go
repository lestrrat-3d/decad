package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/lestrrat-go/option/v3"
)

// This file is docs/sweep-design.md §16's mitred polyline sweep: the two
// options that select it, the gates Table SM states ahead of the build, the
// payload that records it, its placement, and its exact-restatement
// tessellation (Table DM rows DM2 and DM7). sweep_mitre_build.go owns the
// §16.3 construction, Table BM's topology and §16.6's four readings.

type identMitredJoins struct{}

type identSectionScale struct{}

// WithMitredJoins admits a LineTo-only path whose internal joins are corners:
// the two spans meeting at a join are each cut on one join plane and share the
// section polygon on it (docs/sweep-design.md §16). The profile must be made
// of straight lines only. Passing it twice is ErrDegenerate.
func WithMitredJoins() SweepOption {
	return sweepOption{option.New(identMitredJoins{}, struct{}{})}
}

// WithSectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile, so a branch whose radius is r_k at path point k passes
// r_k / r_0. Each factor must be a positive, finite Dimensionless value, and
// the count must equal the path's segment count. On a path of two or more
// spans it requires [WithMitredJoins] (docs/sweep-design.md §16.2).
func WithSectionScale(factors ...units.Value) SweepOption {
	return sweepOption{option.New(identSectionScale{}, append([]units.Value(nil), factors...))}
}

// sweepConfig is what Sweep's options resolve to.
type sweepConfig struct {
	surfaceResult bool
	mitred        bool
	scaled        bool
	// factors is WithSectionScale's validated factors as base-unit floats,
	// one per path segment; nil when no scale was passed.
	factors []float64
}

// validateSectionScale is Table SM row SM3 over one WithSectionScale payload:
// kind, finiteness and sign per factor in order, then the count against the
// path's own segment count.
func validateSectionScale(raw []units.Value, segments int) ([]float64, error) {
	out := make([]float64, len(raw))
	for k, f := range raw {
		if f.Kind() != units.Dimensionless {
			return nil, fmt.Errorf(`%w: section scale factor %d must be dimensionless, got %s`, ErrUnitKind, k, f.Kind())
		}
		v := f.Base()
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, fmt.Errorf(`%w: section scale factor %d is not finite`, ErrNotFinite, k)
		}
		if v <= 0 {
			return nil, fmt.Errorf(`%w: section scale factor %d must be positive, got %g`, ErrDegenerate, k, v)
		}
		out[k] = v
	}
	if len(out) != segments {
		return nil, fmt.Errorf(`%w: WithSectionScale states %d factors for a path of %d segments`, ErrDegenerate, len(out), segments)
	}
	return out, nil
}

// mitredSweepLoops is Table SM row SM2: every segment of every loop must be a
// LineSeg walked whole, so each section vertex is a recorded point. It
// returns the recorded walk-start point of every segment, loop by loop, and
// the per-loop index arrays the cap triangulator reads. A LineSeg trimmed at
// a cut parameter, or a junction whose two recorded points disagree, states
// no exact polygon vertex and is refused as unsupported.
func mitredSweepLoops(profile ProfileRecord) ([]Point2, [][]int, error) {
	loops := append([]LoopRecord{profile.Outer}, profile.Holes...)
	var pts []Point2
	var loopIdx [][]int
	for i, loop := range loops {
		n := len(loop.Segments)
		starts := make([]Point2, n)
		ends := make([]Point2, n)
		for j, raw := range loop.Segments {
			segment, err := normalizeSegment(raw)
			if err != nil {
				return nil, nil, err
			}
			line, ok := segment.(LineSeg)
			if !ok {
				return nil, nil, fmt.Errorf(`%w: a mitred or scaled sweep supports line profile segments only; loop %d segment %d is %T`, ErrUnsupported, i, j, segment)
			}
			switch {
			case line.TStart == 0 && line.TEnd == 1:
				starts[j], ends[j] = line.Start, line.End
			case line.TStart == 1 && line.TEnd == 0:
				starts[j], ends[j] = line.End, line.Start
			default:
				return nil, nil, fmt.Errorf(`%w: a mitred or scaled sweep needs whole profile lines; loop %d segment %d is trimmed`, ErrUnsupported, i, j)
			}
		}
		idx := make([]int, n)
		for j := range n {
			if ends[j] != starts[(j+1)%n] {
				return nil, nil, fmt.Errorf(`%w: loop %d segment %d does not end exactly where the next one starts`, ErrUnsupported, i, j)
			}
			idx[j] = len(pts)
			pts = append(pts, starts[j])
		}
		loopIdx = append(loopIdx, idx)
	}
	return pts, loopIdx, nil
}

// mitredSweepPreflight is Table SM row SM10, decided from counts alone before
// any rational is built: the span ceiling, then F·(F−1)/2 over Table BM's
// held triangle count F against the fixed facet-pair ceiling.
func mitredSweepPreflight(loopIdx [][]int, spans int) error {
	if spans > maxSweepSpansPerCall {
		return fmt.Errorf(`%w: a mitred sweep exceeds the fixed span ceiling of %d`, ErrUnsupported, maxSweepSpansPerCall)
	}
	segments := uint64(0)
	for _, idx := range loopIdx {
		segments += uint64(len(idx))
	}
	holes := uint64(len(loopIdx) - 1)
	// A polygon with h holes and S boundary vertices triangulates into
	// S + 2h − 2 triangles; each cap holds one such triangulation.
	if segments > math.MaxUint32 {
		return fmt.Errorf(`%w: the mitred sweep's triangle count overflows`, ErrUnsupported)
	}
	capTris := segments + 2*holes - 2
	walls := 2 * uint64(spans) * segments
	pairs, ok := proofbound.WallChoose2(walls + 2*capTris)
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the mitred sweep crossing audit exceeds its fixed facet-pair ceiling`, ErrUnsupported)
	}
	return nil
}

// sweepMitred runs Table SM's gates in §5's order and builds the body. The
// options were validated by the caller (SM3, SM9's surface-result arm), and
// the profile is already authenticated.
func sweepMitred(ctx context.Context, d *Document, profile ProfileRecord, plane PlaneRecord, path *Path, cfg sweepConfig) (*Body, error) {
	segments := path.Segments()
	for k, segment := range segments {
		if _, ok := segment.(LineTo); !ok {
			return nil, fmt.Errorf(`%w: a mitred or scaled sweep follows LineTo segments only; path segment %d is %T`, ErrUnsupported, k, segment)
		}
	}
	if cfg.scaled && !cfg.mitred && len(segments) > 1 {
		return nil, fmt.Errorf(`%w: WithSectionScale on a path of %d spans requires WithMitredJoins`, ErrUnsupported, len(segments))
	}
	if err := validateSweepPathStart(path, plane); err != nil {
		return nil, err
	}
	if samePathPoint(path.Start(), path.End()) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}
	_, loopIdx, err := mitredSweepLoops(profile)
	if err != nil {
		return nil, err
	}
	if err := mitredSweepPreflight(loopIdx, len(segments)); err != nil {
		return nil, err
	}
	factors := cfg.factors
	if factors == nil {
		factors = make([]float64, len(segments))
		for k := range factors {
			factors[k] = 1
		}
	}
	return evalMitredSweep(ctx, d, d.nextProducerID(), mitredSweepPayload{
		profile: profile,
		plane:   plane,
		path:    path,
		factors: factors,
		xform:   r3.Identity(),
	})
}

// mitredSweepPayload is the evaluator's record of a mitred polyline sweep
// (docs/sweep-design.md §16): the authenticated profile and its plane, the
// path, one scale factor per span and the accumulated placement — what a
// placement re-runs §16.3 from — plus the build that record produced.
//
// exact holds every vertex as the rational the construction denotes, placed
// exactly under xform; verts is each rounded once to the nearest float, and
// delta is the largest 3D gap between the two. tris is the globally oriented
// held triangle set, walls first in (span, loop, segment) order, then capStart
// and capEnd; triFace names each triangle's face as an index into faceRoles.
type mitredSweepPayload struct {
	profile ProfileRecord
	plane   PlaneRecord
	path    *Path
	factors []float64
	xform   r3.Transform

	exact     []sweepRatVec
	verts     []r3.Vec
	tris      [][3]int
	triFace   []int
	faceRoles []string
	delta     float64
}

func (mp mitredSweepPayload) transform() r3.Transform { return mp.xform }

// axialDelta is every held vertex's displacement; a ToFace stop against
// either cap inherits it.
func (mp mitredSweepPayload) axialDelta() float64 { return mp.delta }

// placed re-runs §16.3 from the record under the composed motion (Table DM
// row DM7): the construction rebuilds the unplaced rationals, applies the
// motion to them exactly, rounds once, re-decides the shell orientation and
// re-runs the crossing audit, so delta never accumulates across placements.
func (mp mitredSweepPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	next := mitredSweepPayload{
		profile: mp.profile,
		plane:   mp.plane,
		path:    mp.path,
		factors: mp.factors,
		xform:   composed,
	}
	return evalMitredSweep(ctx, d, ref, next)
}

// tessellateMitredSweep is Table DM row DM2's exact restatement: the held
// triangles of Table BM, with every face's source bound delta. A tolerance
// below delta asks for a mesh closer to the body than its held vertices are,
// which no restatement can give (docs/tessellation-design.md §7's rule). The
// occupied-volume proof is the swept-volume allowance at delta over the
// perturbed area, because the held set is the exact body with every vertex
// moved by at most delta; the boundary proof is the build's own crossing
// audit, so the mesh needs no facet-contact audit of its own.
func tessellateMitredSweep(ctx context.Context, b *Body, mp mitredSweepPayload, chord float64) (*Mesh, error) {
	if chord < mp.delta {
		return nil, fmt.Errorf(`%w: a mitred sweep restates its held vertices, which sit up to %g mm from the body; a tolerance of %g mm is finer than that`, ErrUnsupported, mp.delta, chord)
	}
	faceOfRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			faceOfRole[o.Role] = f
		}
	}
	src := make([]*Face, len(mp.tris))
	for k, slot := range mp.triFace {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		f, ok := faceOfRole[mp.faceRoles[slot]]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, mp.faceRoles[slot])
		}
		src[k] = f
	}
	if err := tessellation.RequireClosedMesh(mp.tris); err != nil {
		return nil, fmt.Errorf(`%w: the mitred sweep's held triangle set is not a closed mesh`, ErrUnsupported)
	}
	mesh := &Mesh{
		vertices:  append([]r3.Vec(nil), mp.verts...),
		triangles: append([][3]int(nil), mp.tris...),
		source:    src,
	}
	for _, f := range src {
		mesh.setFaceBound(f, mp.delta)
	}
	mesh.bound = mp.delta
	slack := 0.0
	for _, t := range mp.tris {
		slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(mp.verts[t[0]], mp.verts[t[1]], mp.verts[t[2]], mp.delta))
	}
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, mp.verts, mp.tris, mp.delta)
	if err != nil {
		return nil, err
	}
	volSymDiff := proofbound.SweptVolumeAllow(mp.delta, areaUpper)
	if !finiteMeasurementValues(slack, volSymDiff) {
		return nil, fmt.Errorf(`%w: the mitred sweep states no finite proof for a required mesh bound`, ErrUnsupported)
	}
	mesh.areaSlack = slack
	mesh.volSymDiff = volSymDiff
	mesh.symDiffOK = true
	return mesh, nil
}
