using System.IO;
using UnityEngine;

public class ImageParser
{
    public ImageFile LoadPNG(string pngPath)
    {
        ImageFile image = new ImageFile();
        string path = Path.Combine(Application.streamingAssetsPath, pngPath);
        byte[] bytes = File.ReadAllBytes(path);
        image.texture = new Texture2D(2, 2, TextureFormat.R8, false);
        image.texture.LoadImage(bytes);
        image.texture.Apply();

        image.width = image.texture.width;
        image.height = image.texture.height;
        image.probGrid = new float[image.width, image.height];

        var pix = image.texture.GetPixels32();
        for (int y = 0; y < image.width; y++)
            for (int x = 0; x < image.height; x++)
                image.probGrid[x, y] = pix[y * image.width + x].r / 255f;

        return image;
    }

    public ImageFile LoadPGM(string pgmPath)
    {
        ImageFile image = new ImageFile();
        string path = Path.Combine(Application.streamingAssetsPath, pgmPath);
        using var fs = new FileStream(path, FileMode.Open, FileAccess.Read);
        using var reader = new BinaryReader(fs);

        string magic = ReadToken(reader);
        if (magic != "P5")
            throw new System.Exception("Unsupported PGM Format" + magic);

        image.width = int.Parse(ReadToken(reader));
        image.height = int.Parse(ReadToken(reader));
        int maxVal = int.Parse(ReadToken(reader));

        byte[] pix = reader.ReadBytes(image.width * image.height);
        image.probGrid = new float[image.width, image.height];

        image.texture = new Texture2D(image.width, image.height, TextureFormat.R8, false);
        image.texture.LoadRawTextureData(pix);
        image.texture.Apply();

        for(int y = 0; y < image.height; y++)
            for(int x = 0; x < image.width; x++)
                image.probGrid[x,y] = pix[y*image.width+x]/(float)maxVal;
        return image;
    }


    string ReadToken(BinaryReader reader)
    {
        char c;
        // 1. 공백·줄바꿈 건너뛰기, 주석(#) 처리
        do
        {
            c = (char)reader.ReadByte();
            if (c == '#')
            {
                // 줄 끝까지 건너뛰기
                while ((char)reader.ReadByte() != '\n') { }
            }
        } while (char.IsWhiteSpace(c));

        // 2. 토큰 읽기 시작
        var token = "";
        do
        {
            token += c;
            c = (char)reader.ReadByte();
        } while (!char.IsWhiteSpace(c));

        return token;
    }
}
