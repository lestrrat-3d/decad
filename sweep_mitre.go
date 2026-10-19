package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/featureoption"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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
	return sweepOptionValue{featureoption.WithMitredJoins()}
}

// WithSectionScale states one dimensionless factor per path segment, in path
// order: factors[k] is the ratio of the section at the END of span k to the
// authored profile, so a branch whose radius is r_k at path point k passes
// r_k / r_0. Each factor must be a positive, finite Dimensionless value, and
// the count must equal the path's segment count. On a path of two or more
// spans it requires [WithMitredJoins] (docs/sweep-design.md §16.2).
func WithSectionScale(factors ...units.Value) SweepOption {
	return sweepOptionValue{featureoption.WithSectionScale(factors...)}
}

// mitredSweepPreflight applies SM10's span and facet-pair ceilings.
func mitredSweepPreflight(loopIdx [][]int, spans int) error {
	return sweepmitre.Preflight(loopIdx, spans, compositesweep.MaxSpansPerCall)
}

// sweepMitred runs Table SM's gates in §5's order and builds the body. The
// options were validated by the caller (SM3, SM9's surface-result arm), and
// the profile is already authenticated.
func sweepMitred(ctx context.Context, d *Document, profile profileRecord, plane planeRecord, path *sweepinput.Path, cfg featureoption.SweepConfig) (*Body, error) {
	segments := pathSegments(path)
	for k, segment := range segments {
		if _, ok := segment.(LineTo); !ok {
			return nil, fmt.Errorf(`%w: a mitred or scaled sweep follows LineTo segments only; path segment %d is %T`, ErrUnsupported, k, segment)
		}
	}
	if cfg.Scaled && !cfg.Mitred && len(segments) > 1 {
		return nil, fmt.Errorf(`%w: WithSectionScale on a path of %d spans requires WithMitredJoins`, ErrUnsupported, len(segments))
	}
	if err := sweepinput.ValidateSweepPathStart(path, plane); err != nil {
		return nil, err
	}
	if sweepinput.SamePathPoint(path.Start, path.End) {
		return nil, fmt.Errorf(`%w: closed sweep paths are not implemented`, ErrUnsupported)
	}
	_, loopIdx, err := sweepmitre.Loops(profile.Outer, profile.Holes)
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
	return bodyResult(sweepmitre.Build(ctx, envOf(documentNode(d), d.nextProducerID()), mitredSweepPayload{
		Profile: profile,
		Plane:   plane,
		Path:    path,
		Factors: factors,
		Xform:   r3.Identity(),
	}))
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
	if chord < mp.Delta {
		return nil, fmt.Errorf(`%w: a mitred sweep restates its held vertices, which sit up to %g mm from the body; a tolerance of %g mm is finer than that`, ErrUnsupported, mp.Delta, chord)
	}
	faceOfRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			faceOfRole[o.Role] = f
		}
	}
	src := make([]*topo.Face, len(mp.Tris))
	for k, slot := range mp.TriFace {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		f, ok := faceOfRole[mp.FaceRoles[slot]]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, mp.FaceRoles[slot])
		}
		src[k] = faceNode(f)
	}
	if err := tessellation.RequireClosedMesh(mp.Tris); err != nil {
		return nil, fmt.Errorf(`%w: the mitred sweep's held triangle set is not a closed mesh`, ErrUnsupported)
	}
	mesh := bornMesh(&Mesh{
		n: topo.Mesh{
			Vertices:  append([]r3.Vec(nil), mp.Verts...),
			Triangles: append([][3]int(nil), mp.Tris...),
			Source:    src,
		},
	})
	topobuild.SetVertexBounds(&mesh.n, mp.VertexBound)
	mesh.n.Bound = mp.Delta
	slack := 0.0
	for _, t := range mp.Tris {
		slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(mp.Verts[t[0]], mp.Verts[t[1]], mp.Verts[t[2]], mp.Delta))
	}
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, mp.Verts, mp.Tris, mp.Delta)
	if err != nil {
		return nil, err
	}
	volSymDiff := proofbound.SweptVolumeAllow(mp.Delta, areaUpper)
	if !measurement.FiniteValues(slack, volSymDiff) {
		return nil, fmt.Errorf(`%w: the mitred sweep states no finite proof for a required mesh bound`, ErrUnsupported)
	}
	mesh.n.AreaSlack = slack
	mesh.n.VolSymDiff = volSymDiff
	mesh.n.SymDiffOK = true
	return mesh, nil
}
