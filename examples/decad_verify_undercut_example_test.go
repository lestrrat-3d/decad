package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Mixed undercut coverage (proposal §7, §12): a rounded cap-blend corner
// carries one flat chamfer patch a +x pull provenly opposes and one circular
// chamfer patch the survey cannot separate from the pull within its own
// proven bound. Coverage reads Partial, never Complete or Undecided, because
// a nonempty confirmed Faces list coexists with membership the survey could
// not finish deciding elsewhere.
func Example_decad_verify_undercuts() {
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	px := s.CreatePoint(100, 0)
	py := s.CreatePoint(0, 100)
	s.CreateLine(o, px)
	s.CreateLine(py, o)
	s.CreateArc(o, px, py)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}

	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	chamfered, err := body.Chamfer(decad.Edges(decad.CreatedBy(decad.CapEnd(body))), units.Millimeters(0.5))
	if err != nil {
		fmt.Printf("failed to chamfer: %s\n", err)
		return
	}

	report, err := chamfered.Document().Verify(context.Background(), decad.WithPullDirection(r3.NewVec(1, 0, 0)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	undercut := report.Bodies[0].Undercut
	fmt.Printf("coverage: %s, confirmed faces: %d, assessment: %s\n",
		undercut.Coverage, len(undercut.Faces), undercut.Assessment)
	// Output:
	// coverage: partial, confirmed faces: 1, assessment: violated
}
