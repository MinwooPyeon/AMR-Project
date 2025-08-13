using UnityEngine;

public static class ColorMapper
{
    /// <summary>
    /// Color -> Probability 매핑
    /// </summary>
    public static float ConvertColorToProb(Color color)
    {
        if (color == Color.white) return 1.0f;
        if (color == Color.black) return 0.0f;
        if (color == Color.red) return 0.66f;
        if (color == Color.blue) return 0.33f;
        if (color == Color.green) return 0.5f;
        
        return 0.25f; // default unknown
    }

    /// <summary>
    /// Probability -> Color 매핑
    /// </summary>
    public static Color ConvertProbToColor(float prob)
    {
        if (prob > 0.9f) return Color.white;
        if (prob < 0.1f) return Color.black;

        if (prob >= 0.55f && prob < 0.7f) return Color.red;    // ~0.6
        if (prob >= 0.3f && prob < 0.37f) return Color.blue;   // ~0.33
        if (prob >= 0.48f && prob < 0.52f) return Color.green; // ~0.5

        return Color.gray; // fallback
    }
}