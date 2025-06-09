// The MIT License
// Copyright © 2023 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// More information here: https://iquilezles.org/articles/sdfrepetition


// https://iquilezles.org/articles/distfunctions
float sdBox( in vec2 p, in vec2 b ) 
{
    vec2 q = abs(p) - b;
    return min(max(q.x,q.y),0.0) + length(max(q,0.0));
}

float shape( in vec2 p, in vec2 id )
{
    float a = 0.25*iTime;
    return sdBox( mat2x2(cos(a),-sin(a),sin(a),cos(a))*p, vec2(0.7,0.4) ) -  0.1;
}

//-----------------------------

float map( in vec2 p )
{
    const float s = 2.0;
    const vec2 rep = vec2(2,1);
    
    int method = int(iTime/4.0) % 3;
    
    // naive
    if( method==0 )
    {
        vec2 id = round(p/s);
        id = clamp(id,-rep,rep); // limited repetition
        vec2 r = p - s*id;
        return shape( r, id );
    }

    // correct
    if( method==1 )
    {
        vec2 id = round(p/s);

        vec2 off = sign(p-s*id);

        float d = 1e20;
        for( int j=0; j<2; j++ )
        for( int i=0; i<2; i++ )
        {
            vec2 rid = id + vec2(i,j)*off;
            rid = clamp(rid,-rep,rep); // limited repetition
            vec2 r = p - s*rid;
            d = min( d, shape(r,rid) );
        }

        return d;
    }
    
    // mirror
    if( method==2 )
    {
        vec2 id = round(p/s);
        id = clamp(id,-rep,rep); // limited repetition
        vec2 r = p - s*id;

        r = vec2( ((int(id.x)&1)==0) ? r.x : -r.x,
                  ((int(id.y)&1)==0) ? r.y : -r.y );

        return shape( r, id );
    }
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    const float sca = 4.25;
	vec2 p = sca*(2.0*fragCoord-iResolution.xy)/iResolution.y;
	
    // sdf
    float d = map(p);
    
    // colorize
    vec3 col = (d>0.0) ? vec3(0.9,0.6,0.3) : vec3(0.65,0.85,1.0);
	col *= 1.0 - exp(-6.0*abs(d));
	col *= 0.8 + 0.2*cos(31.416*d);
	col = mix( col, vec3(1.0), 1.0-smoothstep(0.0,0.035,abs(d)) );

    // distance samples
    vec2 m = vec2(3.5,2.0)*sin( 0.3*iTime*vec2(1.1,1.3)+vec2(0,2));
    d = map( m );
    col = mix(col, vec3(1.0,1.0,0.0), 1.0-smoothstep(0.0, 0.007, abs(length(p-m)-abs(d))-0.015));
    col = mix(col, vec3(1.0,1.0,0.0), 1.0-smoothstep(0.0, 0.007, length(p-m)-0.08));

	fragColor = vec4(col,1.0);
}
