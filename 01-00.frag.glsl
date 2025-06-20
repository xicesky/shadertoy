/* TODO:
    Paste this code into https://www.shadertoy.com/new
    Checkout for material highlighting: https://youtu.be/bdICU2uvOdU?t=1914
*/

/* -----------------------------------------------------------------------------
    Base SDFs
    Source: https://iquilezles.org/articles/distfunctions/
*/

float sdPlaneX(vec3 p) {
    return abs(p.x);
}

float sdPlaneY(vec3 p) {
    return abs(p.y);
}

float sdPlaneZ(vec3 p) {
    return abs(p.z);
}

float sdSphere(vec3 p, float s) {
    return length(p) - s;
}

float sdBox(vec3 p, vec3 b) {
    vec3 d = abs(p) - b;
    return min(max(d.x, max(d.y, d.z)), 0.0) + length(max(d, 0.0));
}

float sdTorus(vec3 p, vec2 t) {
    vec2 q = vec2(length(p.xz) - t.x, p.y);
    return length(q) - t.y;
}

// Vertical cylinder
float sdCylinder(vec3 p, vec2 h) {
    vec2 d = abs(vec2(length(p.xz), p.y)) - h;
    return min(max(d.x, d.y), 0.0) + length(max(d, 0.0));
}

/* -----------------------------------------------------------------------------
    Rotation functions
*/

vec3 rotateX(vec3 p, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(p.x, p.y * c - p.z * s, p.y * s + p.z * c);
}

vec3 rotateY(vec3 p, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(p.x * c + p.z * s, p.y, -p.x * s + p.z * c);
}

vec3 rotateZ(vec3 p, float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return vec3(p.x * c - p.y * s, p.x * s + p.y * c, p.z);
}

/* -----------------------------------------------------------------------------
    Scene distance function
    Notes on orientation:
        x - right
        y - up
        z - forward (further away into the screen)
*/

// Scene result type
#define SRESULT vec2

vec2 composite(vec2 a, vec2 b) {
    // Returns the minimum distance and the index of the object
    return (a.x < b.x) ? a : b;
}

#define ADD_OBJECT(index, objectSDF) \
    current = composite(vec2(objectSDF, index), current)

struct Material {
    int textureIndex;
    vec3 diffuseColor;
};

const Material materialList[8] = Material[8](
    Material(0, 0.2 * vec3(0.0, 0.0, 0.0)),  // No hit color (black)
    Material(1, 0.2 * vec3(0.7, 0.7, 0.7)),  // Floor: Gray checkerboard
    Material(0, 0.2 * vec3(0.2, 1.0, 0.2)),  // Wall
    Material(0, 0.2 * vec3(1.0, 1.0, 1.0)),  // Sphere in the foreground
    Material(0, 0.2 * vec3(0.1, 0.1, 1.0)),  // Torus around the sphere
    Material(0, 0.2 * vec3(0.0, 1.0, 0.0)),  // Sphere in positive Y
    Material(0, 0.2 * vec3(1.0, 1.0, 0.1)),  // Cube rotating around the X-axis
    Material(0, 0.2 * vec3(0.0, 0.0, 0.0))   // DUMMY: Black filler
);

vec2 sdScene(float time, vec3 p) {
    vec2 current = vec2(1e20, 0.0);  // Initialize with a large distance and no object

    // I want the camera to look at (0,2,0) - so just move the scene for now
    p -= vec3(0.0, -2.0, 0.0);  // Move the scene down by 2 units

    // Floor
    ADD_OBJECT(1, sdPlaneY(p - vec3(0.0, -4.0, 0.0)));

    // Left wall
    // ADD_OBJECT(2, sdPlaneX(p - vec3(-4.0, 0.0, 0.0)));

    // Back wall
    // ADD_OBJECT(2, sdPlaneZ(p - vec3(0.0, 0.0, 4.0)));

    // Sphere in the foreground for testing material highlighting
    // ADD_OBJECT(3, sdSphere(p - vec3(0.0, 0.0, 0.0), 4.0));

    // Torus around the sphere
    vec3 torusP = p - vec3(-1.0, 0.0, 0.0);  // Position
    torusP = rotateX(torusP, time * 0.5);
    ADD_OBJECT(4, sdTorus(torusP, vec2(1.5, 0.5)));

    // Cylinder
    // ADD_OBJECT( 4, sdCylinder(p - vec3(0.0, -1.0, 0.0), vec2(1.0, 2.0)) );

    // Sphere in positive Y
    ADD_OBJECT(5, sdSphere(p - vec3(0.0, 2.0, 0.0), 1.0));

    // Cube rotating around the X-axis
    ADD_OBJECT(6, sdBox(rotateX(p - vec3(0.0, 0.0, 0.0), time * -0.5), vec3(1.0, 1.0, 1.0)));

    // == Start domain repetition setup ========================================
    // FIXME: Still artifacts when objects grow...?!?
    // FIXME: Need to be able to offset domain by some amount < size

    const float size = 8.0;                               // Size of the repeating domain
    vec3 pre_instance = round(p / size) * vec3(1, 0, 1);  // Calculate the instance index
    // Direction to the nearest neighbor - combined into one vector
    vec3 pre_combinedNeighbor = sign(p - size * pre_instance);

    // Find minimum distance in this domain or its neighbors
    float d = 1e20;  // Initialize with a large distance
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            // The current instance has some dialing numbers we can use
            vec3 instance = pre_instance + vec3(i, 0, j) * pre_combinedNeighbor;
            vec3 p_i = p - size * instance;  // Position in the current instance

            // Note that your instance is limited to coordinates in the range [-2, 2] in x and z
            // Do NOT place objects outside of this range, or you will get artifacts
            float instDist = length(instance);
            float bounceHeight = 0.5 + 10.0 * (1.0 - exp(-0.1 * instDist));
            float height_div2 = 0.5 + (1.0 + 0.5 * sin(instDist + time * 1.0)) * bounceHeight;

            // Put cylinder on floor
            float y_center = -4.0 /* floor */ + height_div2;

            ADD_OBJECT(4, sdCylinder(p_i - vec3(0.0, y_center, 0.0), vec2(1.0, height_div2)));
        }
    }
    // == End domain repetition setup ==========================================

    return current;
}

/* -----------------------------------------------------------------------------
    Lighting
*/

// A zero that prevents the compiler from inlining stuff
#define ZERO (min(iFrame, 0))

// Calculate the normal vector at a point in the scene
#define NORMALS_HACK 1

vec3 nScene(float time, vec3 pos) {
    const float h = 0.0005;  // replace by an appropriate value
#if NORMALS_HACK == 0
    // Source: https://iquilezles.org/articles/normalsSDF/
    vec2 e = vec2(1.0, -1.0) * 0.5773 * h;
    return normalize(
        e.xyy * sdScene(time, pos + e.xyy).x +
        e.yyx * sdScene(time, pos + e.yyx).x +
        e.yxy * sdScene(time, pos + e.yxy).x +
        e.xxx * sdScene(time, pos + e.xxx).x);
#else
    // Source: https://iquilezles.org/articles/normalsSDF/
    vec3 n = vec3(0.0);
    for (int i = ZERO; i < 4; i++) {
        vec3 e = 0.5773 * (2.0 * vec3((((i + 3) >> 1) & 1), ((i >> 1) & 1), (i & 1)) - 1.0);
        n += e * sdScene(time, pos + e * h).x;
    }
    return normalize(n);
#endif
}

float ambientOcclusion(float time, vec3 pos, vec3 normal) {
    // Source: https://www.shadertoy.com/view/Xds3zN
    // and https://iquilezles.org/articles/nvscene2008/rwwtt.pdf
    float occ = 0.0;
    float sca = 1.0;
    for (int i = ZERO; i < 5; i++) {
        float h = 0.01 + 0.12 * float(i) / 4.0;
        float d = sdScene(time, pos + h * normal).x;
        occ += (h - d) * sca;
        sca *= 0.95;
    }
    return clamp(1.0 - 3.0 * occ, 0.0, 1.0);
}

float softshadow(float time, vec3 ro, vec3 rd, float mint, float maxt, float w) {
    float res = 1.0;
    float ph = 1e20;
    float t = mint;
    for (int i = 0; i < 256 && t < maxt; i++) {
        float h = sdScene(time, ro + rd * t).x;
        if (h < 0.001)
            return 0.0;
        float y = h * h / (2.0 * ph);
        float d = sqrt(h * h - y * y);
        res = min(res, d / (w * max(0.0, t - y)));
        ph = h;
        // t += h;  // This is the correct way
        t += clamp(h, 0.0, 8.0);  // Just a hack to avoid artifacts due to our DR stuff
    }
    return res;
}

// Note: This does not seem to work well with intersecting objects
float softshadow2(float time, vec3 ro, vec3 rd, float mint, float maxt, float w) {
    // Source: https://iquilezles.org/articles/rmshadows/
    float res = 1.0;
    float t = mint;
    for (int i = 0; i < 128 && t < maxt; i++) {
        float h = sdScene(time, ro + t * rd).x;
        res = min(res, h / (w * t));
        t += clamp(h, 0.005, 0.50);
        if (res < -1.0 || t > maxt) break;
    }
    res = max(res, -1.0);
    return 0.25 * (1.0 + res) * (1.0 + res) * (2.0 - res);
}

/* -----------------------------------------------------------------------------
    Material coloring / texturing
*/

float maxComponent(vec3 v) {
    return max(max(v.x, v.y), v.z);
}

vec3 colorNormal(vec3 normalVector) {
    // Generate a color vector based on the normal vector
    // normalVector = normalize(normalVector);
    // normalVector = normalize(vec3(0,0,-1));
    /*  Hints:
        Pale Red is positive X
        Cyan is negative X
        Pale Green is positive Y
        Purple is negative Y
        Pale Blue is positive Z     (how are you seeing a face pointing away from you?)
        Yellow is negative Z
    */
    // return normalVector * 0.5 + 0.5; // Normalize to [0, 1] range
    return (normalVector + 1.0) / 2.0;
}

// Lookup material
Material lookupMaterial(float index) {
    int i = int(clamp(index, 0.0, float(materialList.length() - 1)));
    return materialList[i];
}

// Basic checkerboard pattern, used for debugging
vec3 checkerboard(vec2 p, vec3 color1, vec3 color2) {
    float c = mod(floor(p.x) + floor(p.y), 2.0);
    return c < 1.0 ? color1 : color2;
}

// Filtered checkerboard pattern
// Original: https://iquilezles.org/articles/checkerfiltering
vec3 checkersGradBox(in vec2 p, in vec2 dpdx, in vec2 dpdy, vec3 color1, vec3 color2) {
    // filter kernel
    vec2 w = abs(dpdx) + abs(dpdy) + 0.001;
    // analytical integral (box filter)
    vec2 i = 2.0 * (abs(fract((p - 0.5 * w) * 0.5) - 0.5) - abs(fract((p + 0.5 * w) * 0.5) - 0.5)) / w;
    // xor pattern
    return mix(color1, color2, 0.5 - 0.5 * i.x * i.y);
}

vec3 calcMaterialColor(float index, Material material, vec3 position) {
    switch (material.textureIndex) {
        case 0:                            // No texture
            return material.diffuseColor;  // Return the color directly
        case 1:                            // Checkerboard texture based on xz plane
            return checkerboard(position.xz, vec3(0.0, 0.0, 0.0), material.diffuseColor);
        default:
            return vec3(0.0);  // Default case, shouldn't happen
    }
}

vec3 distanceFade(vec3 c, vec3 fadeColor, float distance) {
    // Apply a fade effect based on distance
    vec3 fade = exp(distance > 0.0 ? distance * -0.0002 * vec3(1.0, 2.0, 4.0) : vec3(0.0));
    return mix(fadeColor, c, fade);
}

/* -----------------------------------------------------------------------------
    Complete ray rendering output
*/

/* Sets rendering mode.
    0: Standard (material based) coloring
    1: Analysis mode: Color by iterations and distance
    2: Analysis mode: Color by normals
*/
#define RENDER_MODE 0

vec3 colorRay(
    float time,
    bool hit,
    float objectIndex,
    vec3 rayDirection,
    float distance,
    int iteration,
    int maxIteration,
    vec3 position) {
#if RENDER_MODE == 0
    vec3 normal = nScene(time, position);
    // Standard material based coloring
    // Hit color: Look up material based on object index
    Material material = lookupMaterial(objectIndex);
    vec3 materialColor = calcMaterialColor(objectIndex, material, position);
    // materialColor = materialColor * (0.4 + 0.6 * colorNormal(normal)) ; // Looks funny
    // materialColor = mix(materialColor, colorNormal(normal)*0.8, 0.5); // Mix normal color with material color, kinda iridescent

    vec3 sunDirection = normalize(vec3(0.6, 0.6, 0.8));

    // Add up lights
    vec3 color = vec3(0.0);

    float occlusionFactor = ambientOcclusion(time, position, normal);

    // One block per light source
    // Note that a light intensity of 1.0 is not very bright in our HDR setup
    // Direct sharp sunlight is around 100.0, cloudy sky is around 10.0
    {
        // Ambient light - very simple, but keep value low
        color += materialColor * 0.1 * occlusionFactor;
    }
    {
        // "Sun": Parallel directional light
        // Not using occlusion factor here, tip in https://iquilezles.org/articles/outdoorslighting/
        vec3 lightDirection = sunDirection;
        float shadow = softshadow(time, position, lightDirection, 0.01, 60.0, 0.3);

        // Diffuse
        float diffuseFactor = clamp(dot(normal, lightDirection), 0.0, 1.0) * shadow;
        color += 1.0 * materialColor * diffuseFactor;
        // color += 0.5 * vec3(1.0, 1.0, 1.0) * diffuseFactor;  // debug

        // Specular: Don't quite understand this code from IQ yet
        vec3 hal = normalize(lightDirection - rayDirection);
        float specular = pow(clamp(dot(normal, hal), 0.0, 1.0), 16.0);
        specular *= diffuseFactor;
        specular *= 0.04 + 0.96 * pow(clamp(1.0 - dot(hal, lightDirection), 0.0, 1.0), 5.0);
        color += 60.0 * specular * vec3(1.30, 1.00, 0.70);
    }
    {
        // "Sky"
        vec3 lightDirection = vec3(0.0, 1.0, 0.0);
        float lightIntensity = clamp(dot(normal, lightDirection), 0.0, 1.0);
        float shadow = softshadow(time, position, lightDirection, 0.01, 80.0, 0.1);
        color += 1.0 * materialColor * lightIntensity * shadow * occlusionFactor;
    }
    {
        // Indirect light from the sun
        vec3 lightDirection = sunDirection * vec3(-1.0, 0.0, -1.0);
        float lightIntensity = clamp(dot(normal, lightDirection), 0.0, 1.0);
        color += 0.1 * materialColor * lightIntensity * occlusionFactor;
    }
    {
        // Sub-surface scattering: Simulates light bouncing inside the material
        float dif = pow(clamp(1.0 + dot(normal, rayDirection), 0.0, 1.0), 2.0);
        color += 2.5 * materialColor * dif * occlusionFactor;
    }

    // float fre = clamp(1.0+dot(normal,lightDirection),0.0,1.0); // Fresnel effect ? Wat? need ray direction...

    // Apply some fading based on distance ("fog")
    color = distanceFade(color, vec3(5.0), distance);

    // See also: https://iquilezles.org/articles/outdoorslighting/
    color = 1.2 * color;                  // TEMP brightness correction for HDR
    color = color / (color + vec3(1.0));  // Basic "Reinhard" HDR tone mapping
    color = pow(color, vec3(1.0 / 2.2));  // Gamma correction - screen dependent

    // Just for lighting debugging
    // if (maxComponent(color) > 0.5) {
    //     return vec3(1.0,0.0,0.0);
    // }

    return color;
#elif RENDER_MODE == 1
    // Analysis mode: Color by iterations and distance
    if (!hit) {
        // No hit color: Blue to purple gradient based on iteration
        float f = clamp(float(iteration) / float(maxIteration), 0.0, 1.0);
        return vec3(0.0, 0.0, 1.0) * (1.0 - f) +
            vec3(0.5, 0.0, 1.0) * f;
    } else {
        // Hit color: Black sphere with some fading to white based on distance
        return distanceFade(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0), distance);
    }
#elif RENDER_MODE == 2
    // Analysis mode: Color by normals
    if (hit) {
        vec3 normal = nScene(time, position);
        return colorNormal(normal);
    } else {
        return vec3(0.0, 0.0, 0.0);
    }
#else
    // Default case: You don't want this
    return vec3(0.0, 0.0, 0.0);
#endif
}

/* -----------------------------------------------------------------------------
    Raymarching
*/

/* rayMarch():
    rayInitialPosition: The starting point of the ray in world space.
    rayDirection: The direction of the ray, normalized.
*/
vec3 rayMarch(float time, vec3 rayInitialPosition, vec3 rayDirection) {
    float t = 0.0;                     // Distance along the ray
    const float maxDistance = 1000.0;  // Maximum distance to march
    const float minDistance = 0.001;   // Minimum distance to consider a hit
    const int maxSteps = 200;          // Maximum number of steps

    int i;
    for (i = 0; i < maxSteps; i++) {
        vec3 position = rayInitialPosition + t * rayDirection;  // Current point along the ray
        vec2 sceneResult = sdScene(time, position);             // Find distance to the nearest surface in the scene
        float d = sceneResult.x;                                // Distance to the nearest surface

        if (d < minDistance)  // If we are close enough to the surface
            return colorRay(time, true, sceneResult.y, rayDirection, t, i, maxSteps, position);

        t += d;                      // Move along the ray by the distance
        if (t > maxDistance) break;  // Stop if we exceed max distance
    }

    return colorRay(time, false, -1.0, rayDirection, maxDistance, i, maxSteps, vec3(0.0));  // No hit
}

/* -----------------------------------------------------------------------------
    Debugging / helper functions
*/

vec3 sdfVisualize(float time, vec2 uv) {
    // Scaling factor: "Distance" to cover from the center of the screen to the top or bottom edge
    const float scale = 8.5;  // Adjust as needed for your scene

    // We can hold the time parameter of the scene to a fixed value and use t for something else (e.g. y)
    float sceneTime = 0.0;
    float t = 4.0 * sin(time * 0.5);  // Example time variation, can be removed if not needed

    // Select a 2D slice of the 3D SDF
    // E.g. top-down view at y = 0
    vec3 position = vec3(uv * scale, t).xzy;  // Use uv as x and z, y = 0

    // Evaluate the SDF at this position
    float d = sdScene(sceneTime, position).x;

    // Rest ist colorization:
    // Outer and inner color as a base
    vec3 col = (d > 0.0) ? vec3(0.9, 0.6, 0.3) : vec3(0.65, 0.85, 1.0);
    // col *= 1.0 - exp(-6.0*abs(d)); // ???
    col *= 0.8 + 0.2 * cos(31.416 * d);  // "isolines"
    // Object boundaries
    col = mix(col, vec3(1.0), 1.0 - smoothstep(0.0, 0.035, abs(d)));
    return col;
}

/* -----------------------------------------------------------------------------
    Uniforms
*/

// Uniforms provided by the environment (e.g., Shadertoy)

// uniform vec3 iResolution;    // Resolution of the output image
// uniform float iTime;         // Current time in seconds
// uniform vec4 iMouse;         // Mouse position and click state
// uniform int iFrame;          // Current frame number

/* -----------------------------------------------------------------------------
    Main
*/

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    // fragCoord ranges from (0,0) (lower left) to iResolution.xy (upper right)
    // Clip space: (-f,-1) to (f, 1) (same corners)
    // Where f = iResolution.x / iResolution.y
    vec2 uv = (fragCoord.xy * 2. - iResolution.xy) / iResolution.y;

    // Mouse position, scaled to clip space, too
    vec2 mousePos = (iMouse.xy * 2. - iResolution.xy) / iResolution.y;

    vec3 cameraPosition = vec3(0, 0, -10);
    vec3 rayDirection = normalize(vec3(uv, 1));

    // Mouse controls: Vertical rotation
    float verticalAngle = clamp(-mousePos.y * 1.57, -1.1, 1.1);
    cameraPosition = rotateX(cameraPosition, verticalAngle);
    rayDirection = rotateX(rayDirection, verticalAngle);

    // Mouse controls: Horizontal rotation
    float horizontalAngle = clamp(mousePos.x * 3.14, -3.14, 3.14);
    cameraPosition = rotateY(cameraPosition, horizontalAngle);
    rayDirection = rotateY(rayDirection, horizontalAngle);

    // Run the raymarching algorithm
    vec3 color = rayMarch(iTime, cameraPosition, rayDirection);
    // vec3 color = sdfVisualize(iTime, uv);

    // Output to screen
    fragColor = vec4(color, 1);
}
