// Class for dealing with HDR textures

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "rgbe.h"
#include "hdrtexture.h"

HDRtexture::HDRtexture(
char *filename
) : m_valid(false) {

    m_data = NULL ;
    m_hilodata = NULL ;

    FILE *fp = fopen(filename, "rb") ;
    if (!fp) {
        fprintf(stderr, "Error opening file '%s'\n", filename);
        return;
    }

  rgbe_header_info header;
  if (RGBE_ReadHeader(fp, &m_width, &m_height, &header))
    return;

//  img.data = new float[m_width*m_height*3];
  m_data = new unsigned char[m_width*m_height*4];
  if (!m_data)
    return;

//  if (RGBE_ReadPixels_RLE(fp, img.data, img.width, img.height))
  if (RGBE_ReadPixels_Raw_RLE(fp, m_data, m_width, m_height))
    return;

  fclose(fp);

  m_valid = true;
  printf("Loaded HDR image '%s' %d x %d\n", filename, m_width, m_height);
}

HDRtexture::~HDRtexture()
{
  delete [] m_data;
  delete [] m_hilodata;
}

unsigned char *
HDRtexture::GetPixel(int x, int y)
{
  return m_data + ((m_width * (m_height - 1 - y)) + x) * 4;
}

float *
HDRtexture::GetPixelHilo(int x, int y)
{
  return m_hilodata + ((m_width * (m_height - 1 - y)) + x) * 3;
}


// analyze texture to find minimum and maximum intensities
void
HDRtexture::Analyze()
{
  m_max_r = m_max_g = m_max_b = 0.0;
  m_min_r = m_min_g = m_min_b = 1e10;
  unsigned char mine = 255;
  unsigned char maxe = 0;

  unsigned char *ptr = m_data;
  for(int i=0; i<m_width*m_height; i++) {
    unsigned char e = *(ptr+3);
    if (e < mine) mine = e;
    if (e > maxe) maxe = e;

    float r, g, b;
    rgbe2float(&r, &g, &b, ptr);
    if (r > m_max_r) m_max_r = r;
    if (g > m_max_g) m_max_g = g;
    if (b > m_max_b) m_max_b = b;
    if (r < m_min_r) m_min_r = r;
    if (g < m_min_g) m_min_g = g;
    if (b < m_min_b) m_min_b = b;
    ptr += 4;
  }
  fprintf(stderr, "max intensity: %f %f %f\n", double(m_max_r), double(m_max_g), double(m_max_b));
  fprintf(stderr, "min intensity: %f %f %f\n", double(m_min_r), double(m_min_g), double(m_min_b));
  fprintf(stderr, "max e: %d = %f\n", maxe, ldexp(1.0, maxe-128));
  fprintf(stderr, "min e: %d = %f\n", mine, ldexp(1.0, mine-128));

  m_max = m_max_r;
  if (m_max_g > m_max) m_max = m_max_g;
  if (m_max_b > m_max) m_max = m_max_b;
  fprintf(stderr, "max: %f\n", double(m_max));
}

inline float remap(float x, float max)
{
  if (x > max) x = max;
//  return sqrt(x / max);
  return (x / max);
}

// convert from RGBE to short
void
HDRtexture::ConvertHILO()
{
  m_hilodata = new float [m_width*m_height*3];

  unsigned char *src = m_data;
  float *dest = m_hilodata;
  for(int i=0; i<m_width*m_height; i++) {
    float r, g, b;
    rgbe2float(&r, &g, &b, src);

    dest[0] = remap(r, m_max);
    dest[1] = remap(g, m_max);
    dest[2] = remap(b, m_max);

    src += 4;
    dest += 3;
  }
}


GLuint
HDRtexture::Create2DTextureRGBE(GLenum target)
{
  m_target = target;
  GLuint texid;
  glGenTextures(1, &texid);

  glBindTexture(m_target, texid);

  glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(m_target, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
  glTexImage2D(m_target, 0, GL_RGBA, m_width, m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_data);

  return texid;
}

GLuint
HDRtexture::Create2DTextureHILO(GLenum target, bool rg)
{
  m_target = target;
  GLuint texid;
  glGenTextures(1, &texid);

  glBindTexture(m_target, texid);

  glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(m_target, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);

  float *img = new float [m_width * m_height * 2];
  float *src = m_hilodata;
  float *dest = img;
  for (int j=0; j<m_height; j++) {
    for (int i=0; i<m_width; i++) {
      if (rg) {
        *dest++ = src[0];
        *dest++ = src[1];
      } else {
        *dest++ = src[2];
        *dest++ = 0;
      }
      src+=3;
    }
  }

  glTexImage2D(m_target, 0, GL_HILO16_NV, m_width, m_height, 0, GL_HILO_NV, GL_FLOAT, img);
  delete [] img;

  return texid;
}

GLuint
HDRtexture::CreateCubemap(GLint format)
{
  // cross is 3 faces wide, 4 faces high
  int face_width = m_width / 3;
  int face_height = m_height / 4;
  float *face = new float [face_width * face_height * 3];

  GLuint texid;
  m_target = GL_TEXTURE_CUBE_MAP_ARB;
  glGenTextures(1, &texid);
  glBindTexture(m_target, texid);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(m_target, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);

  glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // extract 6 faces

  // positive Y
  float *ptr = face;
  int j ;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), 3 * face_height + j);
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  // positive X
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(i, m_height - (face_height + j + 1));
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  // negative Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(face_width + i, m_height - (face_height + j + 1));
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  // negative X
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width + i, m_height - (face_height + j + 1));
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  // negative Y
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), face_height + j);
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  // positive Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), j);
      float r, g, b;
      rgbe2float(&r, &g, &b, src);
      *ptr++ = r;
      *ptr++ = g;
      *ptr++ = b;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z_ARB, 0, format, face_width, face_height, 0, GL_RGB, GL_FLOAT, face);

  delete [] face;

  return texid;
}


// create a cubemap texture from a 2D image in cross format (thanks to Jonathon McGee)
GLuint
HDRtexture::CreateCubemapRGBE()
{
  // cross is 3 faces wide, 4 faces high
  int face_width = m_width / 3;
  int face_height = m_height / 4;
  unsigned char *face = new unsigned char[face_width * face_height * 4];

  GLuint texid;
  m_target = GL_TEXTURE_CUBE_MAP_ARB;
  glGenTextures(1, &texid);
  glBindTexture(m_target, texid);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(m_target, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);

  glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // extract 6 faces

  // positive Y
  unsigned char *ptr = face;
  int j ;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), 3 * face_height + j);
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  // positive X
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(i, m_height - (face_height + j + 1));
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  // negative Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(face_width + i, m_height - (face_height + j + 1));
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  // negative X
  ptr = face;
  for (j=0; j<face_height; j++) {
      for (int i=0; i<face_width; i++) {
        unsigned char *src = GetPixel(2 * face_width + i, m_height - (face_height + j + 1));
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  // negative Y
  ptr = face;
  for (j=0; j<face_height; j++) {
      for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), face_height + j);
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  // positive Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      unsigned char *src = GetPixel(2 * face_width - (i + 1), j);
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
      *ptr++ = *src++;
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z_ARB, 0, GL_RGBA, face_width, face_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, face);

  delete [] face;

  return texid;
}

GLuint
HDRtexture::CreateCubemapHILO(bool rg)
{
  // cross is 3 faces wide, 4 faces high
  int face_width = m_width / 3;
  int face_height = m_height / 4;
  float *face = new float [face_width * face_height * 2];

  GLuint texid;
  m_target = GL_TEXTURE_CUBE_MAP_ARB;
  glGenTextures(1, &texid);
  glBindTexture(m_target, texid);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexParameteri(m_target, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);

  glTexParameteri(m_target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(m_target, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(m_target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // extract 6 faces

  // positive Y
  float *ptr = face;
  int j ;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(2 * face_width - (i + 1), 3 * face_height + j);
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }
    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y_ARB, 0, GL_HILO16_NV, face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face);

  // positive X
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(i, m_height - (face_height + j + 1));
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }

    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X_ARB, 0, GL_HILO16_NV, face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face);

  // negative Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(face_width + i, m_height - (face_height + j + 1));
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }

    }
  }
  glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z_ARB, 0, GL_HILO16_NV, face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face);

  // negative X
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(2 * face_width + i, m_height - (face_height + j + 1));
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }

    }
  }
  glTexImage2D(
      GL_TEXTURE_CUBE_MAP_NEGATIVE_X_ARB, 0, GL_HILO16_NV,
      face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face);

  // negative Y
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(2 * face_width - (i + 1), face_height + j);
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }

    }
  }
  glTexImage2D(
      GL_TEXTURE_CUBE_MAP_NEGATIVE_Y_ARB, 0, GL_HILO16_NV,
      face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face
  );

  // positive Z
  ptr = face;
  for (j=0; j<face_height; j++) {
    for (int i=0; i<face_width; i++) {
      float *src = GetPixelHilo(2 * face_width - (i + 1), j);
      if (rg) {
        *ptr++ = src[0];
        *ptr++ = src[1];
      } else {
        *ptr++ = src[2];
        *ptr++ = 0;
      }

    }
  }
  glTexImage2D(
      GL_TEXTURE_CUBE_MAP_POSITIVE_Z_ARB, 0, GL_HILO16_NV,
      face_width, face_height, 0, GL_HILO_NV, GL_FLOAT, face
  );

  delete [] face;

  return texid;
}
