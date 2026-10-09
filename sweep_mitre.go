package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/sweep-design.md §16's mitred polyline sweep: the two
// options that select it, the gates Table SM states ahead of the build, the
// payload that records it, its placement, and its exact-restatement
// tessellation (Table DM rows DM2 and DM7). internal/sweepmitre constructs
// §16.3's exact sections; sweep_mitre_build.go adapts them to Table BM's
// topology and §16.6's four readings.

// WithMitredJoins admits a LineTo-only path whose internal joins are corners:
// the two spans meeting at a join are each cut on one join plane and share the
// section polygon on it (docs/sweep-design.md §16). The profile must be made
// of straight lines only. Passing it twice is ErrDegenerate.
func WithMitredJoins() SweepOption {
	return featureoption.WithMitredJoins()
}

// WithSectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile, so a branch whose radius is r_k at path point k passes
// r_k / r_0. Each factor must be a positive, finite Dimensionless value, and
// the count must equal the path's segment count. On a path of two or more
// spans it requires [WithMitredJoins] (docs/sweep-design.md §16.2).
func WithSectionScale(factors ...units.Value) SweepOption {
	return featureoption.WithSectionScale(factors...)
}

// mitredSweepLoops adapts SM2's whole-line profile gate to its record.
func mitredSweepLoops(profile profileRecord) ([]Point2, [][]int, error) {
	return sweepmitre.Loops(profile.Outer, profile.Holes)
}

// mitredSweepPreflight applies SM10's span and facet-pair ceilings.
func mitredSweepPreflight(loopIdx [][]int, spans int) error {
	return sweepmitre.Preflight(loopIdx, spans, compositesweep.MaxSpansPerCall)
}

// sweepMitred runs Table SM's gates in §5's order and builds the body. The
// options were validated by the caller (SM3, SM9's surface-result arm), and
// the profile is already authenticated.
func sweepMitred(ctx context.Context, d *Document, profile profileRecord, plane planeRecord, path *Path, cfg featureoption.SweepConfig) (*Body, error) {
	segments := path.Segments()
	for k, segment := range segments {
		if _, ok := segment.(LineTo); !ok {
			return nil, fmt.Errorf(`%w: a mitred or scaled sweep follows LineTo segments only; path segment %d is %T`, ErrUnsupported, k, segment)
		}
	}
	if cfg.Scaled && !cfg.Mitred && len(segments) > 1 {
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
	factors := cfg.Factors
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
// exactly under xform; verts is each rounded once to the nearest float,
// vertexBound is each vertex's own 3D gap between the two, rounded up, and
// delta is the largest of them. tris is the globally oriented
// held triangle set, walls first in (span, loop, segment) order, then capStart
// and capEnd; triFace names each triangle's face as an index into faceRoles.
type mitredSweepPayload struct {
	profile profileRecord
	plane   planeRecord
	path    *Path
	factors []float64
	xform   r3.Transform

	exact       []sweepRatVec
	verts       []r3.Vec
	vertexBound []float64
	tris        [][3]int
	triFace     []int
	faceRoles   []string
	delta       float64
	// These sums use the unplaced sections and canonical triangle winding.
	// Placements reuse them before deciding the new shell orientation.
	localVol6    *big.Rat
	localMoments [3]*big.Rat
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
		profile:      mp.profile,
		plane:        mp.plane,
		path:         mp.path,
		factors:      mp.factors,
		xform:        composed,
		localVol6:    mp.localVol6,
		localMoments: mp.localMoments,
	}
	return evalMitredSweep(ctx, d, ref, next)
}

// tessellateMitredSweep is Table DM row DM2's exact restatement: the held
// triangles of Table BM, publishing each vertex's own rounding gap as its
// per-vertex bound and every face's source bound as the largest corner gap
// over that face's triangles (docs/faceted-vertex-bounds-design.md §2.1):
// every true face is the exact polygon on the rationals, so each held
// triangle's true piece is the affine triangle on its corners' rationals. A
// tolerance below delta asks for a mesh closer to the body than its held
// vertices are, which no restatement can give (docs/tessellation-design.md
// §7's rule). The
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
	mesh.setVertexBounds(mp.vertexBound)
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
