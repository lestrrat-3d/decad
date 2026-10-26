package stepread_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/stretchr/testify/require"
)

const sample = `ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('a'),'2;1');
ENDSEC;
DATA;
/* a comment */
#1=CARTESIAN_POINT('p''q',(0.,1.5E1,-2));
#2 = VERTEX_POINT('',#1);
#3=ORIENTED_EDGE('',*,*,#2,.T.);
#4=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#5=FOO($,LENGTH_MEASURE(2.5),(#1,#2));
ENDSEC;
END-ISO-10303-21;
`

func TestParseReadsEveryValueKind(t *testing.T) {
	t.Parallel()
	m, err := stepread.Parse([]byte(sample))
	require.NoError(t, err)
	require.Len(t, m.Entities, 5)
	point := m.Entities[1]
	require.Equal(t, "CARTESIAN_POINT", point.Name)
	require.Equal(t, stepread.String("p'q"), point.Params[0])
	require.Equal(t, stepread.List{stepread.Number(0), stepread.Number(15), stepread.Number(-2)}, point.Params[1])
	edge := m.Entities[3]
	require.Equal(t, stepread.List{stepread.String(""), stepread.Omitted{}, stepread.Omitted{}, stepread.Ref(2), stepread.Enum("T")}, edge.Params)
	combined := m.Entities[4]
	require.Empty(t, combined.Name)
	require.Equal(t, []string{"LENGTH_UNIT", "NAMED_UNIT", "SI_UNIT"}, []string{combined.Parts[0].Name, combined.Parts[1].Name, combined.Parts[2].Name})
	require.Equal(t, stepread.List{stepread.Enum("MILLI"), stepread.Enum("METRE")}, combined.Parts[2].Params)
	foo := m.Entities[5]
	require.Equal(t, stepread.Null{}, foo.Params[0])
	require.Equal(t, stepread.Typed{Name: "LENGTH_MEASURE", Values: stepread.List{stepread.Number(2.5)}}, foo.Params[1])
}

func TestParseRefuses(t *testing.T) {
	t.Parallel()
	for name, src := range map[string]string{
		"no data":            "ISO-10303-21;HEADER;ENDSEC;",
		"no endsec":          "DATA;#1=A();",
		"a missing equals":   "DATA;#1 A();ENDSEC;",
		"an open list":       "DATA;#1=A((1,2);ENDSEC;",
		"a bad number":       "DATA;#1=A(1.2.3);ENDSEC;",
		"an unclosed string": "DATA;#1=A('abc);ENDSEC;",
		"a twice defined id": "DATA;#1=A();#1=B();ENDSEC;",
		"no semicolon":       "DATA;#1=A()ENDSEC;",
	} {
		_, err := stepread.Parse([]byte(src))
		require.ErrorIs(t, err, stepread.ErrMalformed, name)
	}
}
