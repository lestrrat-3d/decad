package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func TestLoftQuarterJunctionOneRealCase(t *testing.T) {
	ctx := t.Context()
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 10)
	if err != nil {
		t.Fatal(err)
	}
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, radius := range [2]float64{5, 5.5} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, e := world.CreateSketch(plane)
		if e != nil {
			t.Fatal(e)
		}
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		right := s.CreatePoint(radius, 0)
		top := s.CreatePoint(0, radius)
		s.CreateLine(center, right)
		s.CreateArc(center, right, top)
		s.CreateLine(top, center)
		if _, e = s.Solve(ctx); e != nil {
			t.Fatal(e)
		}
		found := s.Profiles()
		if len(found) != 1 || !found[0].Valid {
			t.Fatalf("producer cap %d: profiles=%d", cap, len(found))
		}
		sections[cap], profiles[cap] = s, found[0]
	}
	doc := New()
	loft, err := Loft(ctx, doc, sections[0], profiles[0],
		sections[1], profiles[1])
	if err != nil {
		t.Fatal(err)
	}
	prismSketch, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := prismSketch.CreateRectangle(2, 1, 6, 3)
	prismSketch.Fix(rect.A)
	if _, err = prismSketch.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	prismProfiles := prismSketch.Profiles()
	if len(prismProfiles) != 1 || !prismProfiles[0].Valid {
		t.Fatalf("prism profiles=%d", len(prismProfiles))
	}
	prism, err := Extrude(ctx, doc, prismSketch, prismProfiles[0],
		Distance{D: units.Millimeters(12), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}

	var total uint64
	operation := &proofbound.WorkBudget{
		StepFn: func() error { total++; return ctx.Err() },
		ErrFn:  ctx.Err,
	}
	wall := proofbound.NewWallWorkBudgetWithOperation(4096, operation)
	source, ok, err := certifyLoftSourceSpanCaps(ctx, wall, prism, loft)
	if err != nil || !ok {
		t.Fatalf("source authentication: ok=%t err=%v steps=%d", ok, err, total)
	}
	work := &loftQuarterWork{budget: wall}
	order, ok, err := certifyLoftCapSourceOrder(ctx, wall, source, work)
	t.Logf("quarter source counts: total=%d decad=%d raw=%d proof=%d pairs=%d",
		total, total-work.raw-work.proof-work.pairs,
		work.raw, work.proof, work.pairs)
	if work.declined {
		d := work.decline
		t.Logf("first raw decline: predicate=%s cap=%d profile=%d loop=%d edge=%d left_source=%d left_kind=%T left_tagged=%t left_is_loft=%t left_segment=%d right_source=%d right_kind=%T right_tagged=%t right_is_loft=%t right_segment=%d",
			d.Predicate, work.cap, d.Profile, d.Loop, d.Edge,
			d.LeftSourceIndex, d.LeftEntity, work.leftTagged,
			work.leftTag.IsB, work.leftTag.Segment,
			d.RightSourceIndex, d.RightEntity, work.rightTagged,
			work.rightTag.IsB, work.rightTag.Segment)
	}
	if err != nil || !ok {
		t.Fatalf("quarter cap proof: ok=%t err=%v", ok, err)
	}
	if work.pairs != 24 {
		t.Fatalf("original pair scans=%d, want 24", work.pairs)
	}
	for cap := range 2 {
		if len(order.quarter[cap]) != 2 {
			t.Fatalf("cap %d: junctions=%d, want 2", cap, len(order.quarter[cap]))
		}
		seen := map[float64]bool{}
		for _, event := range order.quarter[cap] {
			line := event.line.curve.(sectionrecord.LineSeg)
			if line.Start.V != line.End.V || event.appearances == 0 {
				t.Fatalf("cap %d: unverified source occurrence", cap)
			}
			y := line.Start.V
			var want *big.Rat
			switch {
			case cap == 0 && y == 1:
				want = big.NewRat(24, 1)
			case cap == 0 && y == 3:
				want = big.NewRat(16, 1)
			case cap == 1 && y == 1:
				want = big.NewRat(117, 4)
			case cap == 1 && y == 3:
				want = big.NewRat(85, 4)
			default:
				t.Fatalf("cap %d: unexpected line y=%g", cap, y)
			}
			x, yBox := event.witness.Point()
			xlo, xhi := x.Bounds()
			ylo, yhi := yBox.Bounds()
			if xlo == nil || xhi == nil || ylo == nil || yhi == nil ||
				xlo.Sign() <= 0 || ylo.Cmp(new(big.Rat).SetFloat64(y)) > 0 ||
				yhi.Cmp(new(big.Rat).SetFloat64(y)) < 0 ||
				new(big.Rat).Mul(xlo, xlo).Cmp(want) > 0 ||
				new(big.Rat).Mul(xhi, xhi).Cmp(want) < 0 {
				t.Fatalf("cap %d y=%g: root bracket misses x squared=%s", cap, y, want)
			}
			if seen[y] {
				t.Fatalf("cap %d y=%g: duplicate root", cap, y)
			}
			seen[y] = true
		}
	}
	wallOrder, ordered, orderErr := certifyLoftQuarterWallOrder(ctx, wall, source)
	if orderErr != nil || !ordered || wallOrder.prism != prism ||
		wallOrder.loft != loft || wallOrder.arcSegment < 0 ||
		len(wallOrder.wallFaces[0]) == 0 ||
		len(wallOrder.wallFaces[0]) != len(wallOrder.wallFaces[1]) {
		t.Fatalf("quarter wall order: accepted=%t err=%v steps=%d",
			ordered, orderErr, total)
	}
	t.Logf("quarter source and wall order: total=%d remaining=%d wall_faces=%d",
		total, 4096-total, len(wallOrder.wallFaces[0]))
	for _, operands := range [][2]*Body{{prism, loft}, {loft, prism}} {
		total = 0
		envelope, certified, e := certifyLoftQuarterSourceEnvelope(ctx,
			operands[0], operands[1], operation)
		if e != nil || !certified || envelope.visits != total ||
			envelope.visits >= 4096 || envelope.wall.prism != prism ||
			envelope.wall.loft != loft || envelope.wall.arcSegment != wallOrder.arcSegment ||
			envelope.wall.lower != wallOrder.lower || envelope.wall.upper != wallOrder.upper {
			t.Fatalf("shared quarter source: certified=%t err=%v visits=%d steps=%d",
				certified, e, envelope.visits, total)
		}
		for cap := range envelope.order.quarter {
			if len(envelope.order.quarter[cap]) != 2 {
				t.Fatalf("cap %d: envelope junctions=%d", cap,
					len(envelope.order.quarter[cap]))
			}
		}
		total = 0
		wall = proofbound.NewWallWorkBudgetWithOperation(4096, operation)
		_, accepted, e := certifyLoftSourcePartition(ctx,
			operands[0], operands[1], wall)
		if e != nil || accepted {
			t.Fatalf("private source consumer: accepted=%t err=%v steps=%d",
				accepted, e, total)
		}
		total = 0
		wall = proofbound.NewWallWorkBudgetWithOperation(4096, operation)
		_, accepted, e = certifyLoftOutputBoundary(ctx,
			operands[0], operands[1], wall)
		if e != nil || accepted {
			t.Fatalf("private output consumer: accepted=%t err=%v steps=%d",
				accepted, e, total)
		}
	}
	limitErr := errors.New("source operation budget exhausted")
	var limited uint64
	limitedOperation := &proofbound.WorkBudget{
		StepFn: func() error {
			if limited >= 3000 {
				return limitErr
			}
			limited++
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	_, certified, err := certifyLoftQuarterSourceEnvelope(ctx, prism, loft,
		limitedOperation)
	if !errors.Is(err, limitErr) || certified || limited != 3000 {
		t.Fatalf("limited source: certified=%t err=%v steps=%d", certified, err, limited)
	}
	canceled, cancel := context.WithCancel(ctx)
	cancel()
	_, certified, err = certifyLoftQuarterSourceEnvelope(canceled, prism, loft,
		proofbound.NewWorkBudget(canceled))
	if !errors.Is(err, context.Canceled) || certified {
		t.Fatalf("canceled source: certified=%t err=%v", certified, err)
	}
	if err := Remove(prism); err != nil {
		t.Fatal(err)
	}
	for _, operands := range [][2]*Body{{prism, loft}, {loft, prism}} {
		_, certified, err := certifyLoftQuarterSourceEnvelope(ctx,
			operands[0], operands[1], proofbound.NewWorkBudget(ctx))
		if err != nil || certified {
			t.Fatalf("retired source: certified=%t err=%v", certified, err)
		}
	}
}
