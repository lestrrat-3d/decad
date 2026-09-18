package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// thinPlateBody extrudes a 10x10x0.5 mm plate at (x, 0) into doc, a real
// producer whose wall reading is its own 0.5 mm slab — thinner than a 1 mm
// tool. Offsetting by x keeps two such plates in one document box-disjoint,
// so their own pair reads Sound and never adds an unrelated diagnostic.
func thinPlateBody(doc *decad.Document, x float64) (*decad.Body, error) {
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	if err != nil {
		return nil, err
	}
	rect := s.CreateRectangle(x, 0, x+10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, err
	}
	return doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(0.5), Dir: decad.Along})
}

// acceptExceptDesignatedThinWall is proposal §12's positive-match exception
// rule, verbatim: a diagnostic is waived only by matching its exact Code
// AND its exact Body, every other reason is fatal, and the report itself is
// never mutated by the caller's acceptance — a waived DiagWallTooThin stays
// in Diagnostics for whoever reads the report next.
func acceptExceptDesignatedThinWall(ctx context.Context, doc *decad.Document, minimum units.Value, permittedThinBody *decad.Body) error {
	report, err := doc.Verify(ctx, decad.WithMinWallThickness(minimum))
	if err != nil {
		return err
	}
	for _, reason := range report.Diagnostics {
		if reason.Code == decad.DiagWallTooThin && reason.Body == permittedThinBody {
			continue
		}
		return fmt.Errorf("verification %s: %s", reason.Code, reason.Message)
	}
	return nil
}

// Example_decad_verify_exception demonstrates a caller waiving one named,
// expected reason while every other reason stays fatal (proposal §12): a
// thin plate the caller designates permitted passes alone, and adding a
// SECOND thin plate the caller never named makes the same policy reject the
// document, because the exception matches identity, not merely the code.
func Example_decad_verify_exception() {
	minimum := units.Millimeters(1)

	solo := decad.New()
	permitted, err := thinPlateBody(solo, 0)
	if err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	if err := acceptExceptDesignatedThinWall(context.Background(), solo, minimum, permitted); err != nil {
		fmt.Printf("rejected: %s\n", err)
		return
	}
	fmt.Println("accepted: the designated thin wall is the only reason, and it is waived")

	pair := decad.New()
	designated, err := thinPlateBody(pair, 0)
	if err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	if _, err := thinPlateBody(pair, 1000); err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	if err := acceptExceptDesignatedThinWall(context.Background(), pair, minimum, designated); err != nil {
		fmt.Printf("rejected: %s\n", err)
	}
	// Output:
	// accepted: the designated thin wall is the only reason, and it is waived
	// rejected: verification wall_too_thin: the minimum wall thickness is proven below the tool
}
