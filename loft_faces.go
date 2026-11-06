package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/loftfacestrim"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-go/option/v3"
)

const maxLoftFacesSections = loftfacestrim.MaxSections

// LoftFacesOption configures LoftFaces. An option from another package cannot
// implement this sealed tier.
type LoftFacesOption interface {
	option.Interface
	loftFacesOption()
}

type loftFacesOption struct{ option.Interface }

func (loftFacesOption) loftFacesOption() {}

type identConeTangentEnd struct{}

// coneTangentEnd is the value WithConeTangentEnd stores under identConeTangentEnd.
type coneTangentEnd struct {
	cone   *Body
	radial r3.Vec
}

// WithConeTangentEnd ends the loft with a last, constructed section tangent
// to cone along one ruling (agent-docs/loft-face-sections-design.md,
// "Cone-tangent end section").
//
// The cone is a live, unplaced, full straight-meridian Revolve cone about
// world X. radial is the direction of the touching ruling's radius: its X
// must be 0 and its (Y, Z) finite and nonzero. The last section is built from
// the cone, not selected. Its plane is exactly tangent to the cone's exact
// apex and slope, with the source origin at the cone apex, so it fits a
// point-loft source whose plane is that tangent plane. Any other source still
// goes through the ordinary embedding gates, which refuse an inconsistent end.
// The end face carries the cap-end role of the new loft and a rounded Plane
// tag. Cut and Intersect continue from the result as they do without the
// option.
//
// The option is accepted at most once. With it, LoftFaces takes 1 to 31 cap
// faces. The cone must be live in the document of the first section's body.
func WithConeTangentEnd(cone *Body, radial r3.Vec) LoftFacesOption {
	return loftFacesOption{option.New(identConeTangentEnd{}, coneTangentEnd{cone: cone, radial: radial})}
}

// loftFacesConfig is the folded form of a call's LoftFacesOptions.
type loftFacesConfig struct{ tangent *coneTangentEnd }

func foldLoftFacesOptions(opts []LoftFacesOption) (loftFacesConfig, error) {
	var cfg loftFacesConfig
	for _, opt := range opts {
		if opt == nil {
			return cfg, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		if o, ok := opt.(loftFacesOption); ok && o.Interface == nil {
			return cfg, fmt.Errorf(`%w: a zero option names nothing to apply`, ErrDegenerate)
		}
		if _, ok := opt.Ident().(identConeTangentEnd); !ok {
			return cfg, fmt.Errorf(`%w: the LoftFaces option is not a decad LoftFaces option (%T)`, ErrDegenerate, opt)
		}
		end, ok := option.Get[coneTangentEnd](opt)
		if !ok {
			return cfg, fmt.Errorf(`%w: WithConeTangentEnd carries no cone`, ErrDegenerate)
		}
		if cfg.tangent != nil {
			return cfg, fmt.Errorf(`%w: WithConeTangentEnd is accepted at most once`, ErrDegenerate)
		}
		if end.cone == nil {
			return cfg, fmt.Errorf(`%w: WithConeTangentEnd needs a cone`, ErrDegenerate)
		}
		cfg.tangent = &end
	}
	return cfg, nil
}

// LoftFaces builds one solid through ordered complete caps of the same
// authenticated point-loft Split source (agent-docs/loft-face-sections-design.md).
//
// sections are the least slab's toe cap, then heel caps in source order. There
// are 2 to 32 of them, or 1 to 31 with WithConeTangentEnd, which adds a last,
// constructed section. The document is the one owning the first section's
// body, and the result is in the container of that body. A nil context, a
// nil option, a repeated option, a nil cone, a section count outside the
// bounds, a first section that is not a body face, or a radial that
// WithConeTangentEnd cannot use is ErrDegenerate. A cone that is not an
// unplaced exact point cone about world X is ErrUnsupported. A section or cone
// of another document is ErrForeignBody and a retired one ErrRetiredBody.
// Selection failures are those of the section source. A canceled ctx returns
// ctx.Err() before the document changes.
func LoftFaces(ctx context.Context, sections []*Face, opts ...LoftFacesOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf("%w: a nil context cannot control a loft", ErrDegenerate)
	}
	cfg, err := foldLoftFacesOptions(opts)
	if err != nil {
		return nil, err
	}
	lo, hi := 2, maxLoftFacesSections
	if cfg.tangent != nil {
		lo, hi = 1, maxLoftFacesSections-1
	}
	if len(sections) < lo || len(sections) > hi {
		return nil, fmt.Errorf("%w: LoftFaces needs %d to %d cap faces", ErrDegenerate, lo, hi)
	}
	if sections[0] == nil || sections[0].n.Body == nil {
		return nil, fmt.Errorf("%w: LoftFaces section 0 is not a body face", ErrDegenerate)
	}
	d := documentHandle(sections[0].n.Body.Doc)
	if d == nil {
		return nil, fmt.Errorf("%w: LoftFaces section 0 belongs to no document", ErrDegenerate)
	}
	if cfg.tangent != nil {
		if cfg.tangent.cone.n.Doc == nil {
			return nil, fmt.Errorf("%w: the cone belongs to no document", ErrDegenerate)
		}
		if err := d.requireLive(cfg.tangent.cone); err != nil {
			return nil, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var limit *facetedrecord.PointConeLimit
	if cfg.tangent != nil {
		var ok bool
		limit, ok = revolveplan.PointConeSource(&cfg.tangent.cone.n)
		if !ok || !limit.Axis.Canonical() {
			return nil, fmt.Errorf("%w: WithConeTangentEnd needs an unplaced full straight-meridian cone about X",
				ErrUnsupported)
		}
		radial := cfg.tangent.radial
		if !proofbound.FiniteVec(radial) || radial.X != 0 || (radial.Y == 0 && radial.Z == 0) {
			return nil, fmt.Errorf("%w: WithConeTangentEnd needs a finite radial direction with X = 0 and (Y, Z) nonzero",
				ErrDegenerate)
		}
	}
	selected, source, err := collectLoftFaces(d, sections)
	if err != nil {
		return nil, err
	}
	bodies := faceBodies(sections)
	if cfg.tangent != nil {
		bodies = append(bodies, cfg.tangent.cone)
	}
	if err := d.requireSameContainer(bodies); err != nil {
		return nil, err
	}
	defer d.resultIn(d.bodyContainer(bodies[0]))()
	inputs := faceInputs(sections, "section")
	if cfg.tangent == nil {
		return buildLoftFaces(ctx, d, inputs, selected, source, nil)
	}
	tangent, err := loftface.NewTangentEnd(limit.Apex, limit.Slope, cfg.tangent.radial.Y, cfg.tangent.radial.Z)
	if err != nil {
		return nil, fmt.Errorf("%w: %s", ErrUnsupported, err)
	}
	endMap := tangent.Map()
	ring := make([]r3.Vec, len(source.SourceStation))
	for j, station := range source.SourceStation {
		u, v := lfF(station.U), lfF(station.V)
		ring[j] = loftface.RoundPoint(endMap.At(u, v))
	}
	selected = append(selected, revolveloftfaces.LoftFacesSection{Slice: selected[0].Slice, Ring: ring, Constructed: &endMap})
	inputs = append(inputs, OperationInput{Body: cfg.tangent.cone, Role: "tangent", Use: InputReferenced})
	return buildLoftFaces(ctx, d, inputs, selected, source, &tangent)
}

// buildLoftFaces has revolveloftfaces.BuildLoftFaces loft through the
// collected sections, then commits the body.
func buildLoftFaces(ctx context.Context, d *Document, inputs []OperationInput,
	sections []revolveloftfaces.LoftFacesSection,
	source *pointSectionRecord, tangent *loftface.TangentEnd) (*Body, error) {
	ref := d.nextProducerID()
	body, err := bodyResult(revolveloftfaces.BuildLoftFaces(ctx, envOf(documentNode(d), ref), sections, source, tangent))
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := d.commitOperationIn(ctx, nil, OperationLoftFaces, inputs, body); err != nil {
		return nil, err
	}
	return body, nil
}

func faceInputs(faces []*Face, role string) []OperationInput {
	inputs := make([]OperationInput, len(faces))
	for i, face := range faces {
		inputs[i] = OperationInput{Body: bodyHandle(face.n.Body), Role: role, Use: InputReferenced}
	}
	return inputs
}

// collectLoftFaces authenticates the section faces through
// revolveloftfaces.Collect, gating each section's body on d.
func collectLoftFaces(d *Document, faces []*Face) ([]revolveloftfaces.LoftFacesSection, *pointSectionRecord, error) {
	return revolveloftfaces.Collect(faceNodes(faces), func(b *topo.Body) error { return d.requireLive(bodyHandle(b)) })
}

// The LoftFaces cap-map carriers live in internal/loftface.
type loftFaceMap = loftface.Map

func lfF(v float64) *big.Rat { return proofarith.FloatRat(v) }

// lfAdd, lfSub, lfMul and lfDiv return a fresh Rat and read their operands only.
// Operands must be in lowest terms: the result always equals the exact value,
// but is in lowest terms only for reduced operands (a 3/9 operand gives 3/9
// where big.Rat gives 1/3). Every producer in this package reduces: FloatRat
// through lfF, big.NewRat literals, lfOne, lfZero, and
// results of these helpers and of raw Quo, Abs and Neg. The guard
// TestNoRatWritesThroughNumDenom (internal/proof) bars in-place Num and Denom
// writes outside internal/proof. lfDiv by zero panics as Rat.Quo does.
func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }

func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }

func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }

func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }
