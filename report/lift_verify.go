package report

// RefLift maps one instantiation's body and face references to another's, for
// the records that name bodies and faces only.
type RefLift[B1 comparable, F1 any, B2 comparable, F2 any] struct {
	Body func(B1) B2
	Face func(F1) F2
}

func liftSlice[T1, T2 any](in []T1, f func(T1) T2) []T2 {
	if in == nil {
		return nil
	}
	out := make([]T2, len(in))
	for i := range in {
		out[i] = f(in[i])
	}
	return out
}

// diagnostic maps a diagnostic; its pair is rebuilt and its readings carry over.
func (l *RefLift[B1, F1, B2, F2]) diagnostic(d *DiagnosticOf[B1, JointCell]) DiagnosticOf[B2, JointCell] {
	out := DiagnosticOf[B2, JointCell]{
		Code: d.Code, Status: d.Status, Body: l.Body(d.Body), Survey: d.Survey, Reading: d.Reading,
		Observed: d.Observed, ObservedVec: d.ObservedVec, ObservedBox: d.ObservedBox,
		Required: d.Required, At: d.At, Cell: d.Cell, Message: d.Message,
	}
	if d.Pair != nil {
		out.Pair = &DiagnosticPairOf[B2]{A: l.Body(d.Pair.A), B: l.Body(d.Pair.B)}
	}
	return out
}

func (l *RefLift[B1, F1, B2, F2]) diagnostics(in []DiagnosticOf[B1, JointCell]) []DiagnosticOf[B2, JointCell] {
	return liftSlice(in, func(d DiagnosticOf[B1, JointCell]) DiagnosticOf[B2, JointCell] { return l.diagnostic(&d) })
}

func (l *RefLift[B1, F1, B2, F2]) validity(v ValidityResultOf[B1, JointCell]) ValidityResultOf[B2, JointCell] {
	return ValidityResultOf[B2, JointCell]{Outcome: v.Outcome, Diagnostics: l.diagnostics(v.Diagnostics)}
}

func (l *RefLift[B1, F1, B2, F2]) body(b *BodyReportOf[B1, F1, JointCell]) *BodyReportOf[B2, F2, JointCell] {
	if b == nil {
		return nil
	}
	return &BodyReportOf[B2, F2, JointCell]{
		Body: l.Body(b.Body), Status: b.Status, Validity: l.validity(b.Validity), Topology: b.Topology,
		Area: b.Area, Bounds: b.Bounds, Region: b.Region,
		Wall: WallResultOf[B2, JointCell]{Request: b.Wall.Request, Outcome: b.Wall.Outcome, Minimum: b.Wall.Minimum,
			Assessment: b.Wall.Assessment, Diagnostics: l.diagnostics(b.Wall.Diagnostics)},
		Undercut: UndercutResultOf[B2, F2, JointCell]{Request: b.Undercut.Request, Coverage: b.Undercut.Coverage,
			Faces: liftSlice(b.Undercut.Faces, l.Face), Assessment: b.Undercut.Assessment,
			Diagnostics: l.diagnostics(b.Undercut.Diagnostics)},
		ConcaveRadius: ConcaveRadiusResultOf[B2, JointCell]{Outcome: b.ConcaveRadius.Outcome,
			Minimum: b.ConcaveRadius.Minimum, Diagnostics: l.diagnostics(b.ConcaveRadius.Diagnostics)},
		Diagnostics: l.diagnostics(b.Diagnostics),
	}
}

// LiftVerifyReport returns r with every body and face reference mapped by l.
// The request, its wall and undercut pointers and every reading carry over;
// nil and empty slices keep their state. A nil report stays nil.
func LiftVerifyReport[B1 comparable, F1 any, B2 comparable, F2 any](l *RefLift[B1, F1, B2, F2], r *ReportOf[B1, F1, JointCell]) *ReportOf[B2, F2, JointCell] {
	if r == nil {
		return nil
	}
	return &ReportOf[B2, F2, JointCell]{
		Request: r.Request,
		Bodies:  liftSlice(r.Bodies, l.body),
		Interferences: liftSlice(r.Interferences, func(i InterferenceOf[B1]) InterferenceOf[B2] {
			return InterferenceOf[B2]{A: l.Body(i.A), B: l.Body(i.B), Volume: i.Volume}
		}),
		Clearances: liftSlice(r.Clearances, func(c ClearanceOf[B1]) ClearanceOf[B2] {
			return ClearanceOf[B2]{A: l.Body(c.A), B: l.Body(c.B), Gap: c.Gap}
		}),
		Diagnostics: l.diagnostics(r.Diagnostics),
		Status:      r.Status,
	}
}
