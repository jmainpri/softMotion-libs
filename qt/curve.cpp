#include "curve.h"

#if QT_VERSION >= 0x040000
# include <QKeyEvent>
#endif

#include <iostream>
#include <fstream>
#include <string>


using namespace qglviewer;
using namespace std;

Curve::Curve()
{
  _color_f1 = 1.;
  _color_f2 = 1.;
  _color_f3 = 1.;

  for(int i=0; i<3; i++) {
    errorMax.kc[i].a = 0.0;
    errorMax.kc[i].v = 0.0;
    errorMax.kc[i].x = 0.0;
  }
  errorMax.t = 0.0;

  _nbKeyFrames = 0;
  keyFrame_ = NULL;
}

Curve::Curve (const Curve& c)
{
  Frame* myFrame = new Frame();
  _isDraw = c._isDraw;
  _currentKF = c._currentKF;
  _nbKeyFrames = c._nbKeyFrames;
  keyFrame_ = new ManipulatedFrame*[nbKeyFrames()];
  kfi_.setFrame(myFrame);
  kfi_.setLoopInterpolation();
  for (int i=0; i<c._nbKeyFrames; i++) {
      keyFrame_[i] = new ManipulatedFrame();
      keyFrame_[i] = c.keyFrame_[i];
      kfi_.addKeyFrame(keyFrame_[i]);
  }

  _color_f1 = c._color_f1;
  _color_f2 = c._color_f2;
  _color_f3 = c._color_f3;
  path = c.path;
  traj = c.traj;
  discPoint = c.discPoint;
  errorMax = c.errorMax;
}
Curve::~Curve()
{
  if (keyFrame_ != NULL)
  {
    for (int i=0; i<_nbKeyFrames; i++)
      delete keyFrame_[i];

    delete[] keyFrame_;
  }
}

Curve& Curve::operator=(const Curve& curv)
{
  Curve o;

  return o;
}

void Curve::setColor(float f1, float f2, float f3)
{
  _color_f1 = f1;
  _color_f2 = f2;
  _color_f3 = f3;
  return;
}

void Curve::createPath(std::string file)
{
  int i = 0;
  float index, x, y, z, vx, ax;
  ifstream f (file.c_str());
  string line;
  int nbKeyFrames = 0;
  Frame* myFrame = new Frame();

  //   restoreStateFromFile();
  // myFrame is the Frame that will be interpolated.
  
  // Set myFrame as the KeyFrameInterpolator interpolated Frame.
  kfi_.setFrame(myFrame);
  kfi_.setLoopInterpolation();

  nbKeyFrames = 0;
  if (f.is_open())
  {
    while (! f.eof() )
    {
      getline (f,line);
      nbKeyFrames ++;
    }
    f.close();
  }

  //  f.seekg(0, std::ios::beg);
  f.open(file.c_str());
  this->_nbKeyFrames = nbKeyFrames;
  keyFrame_ = new ManipulatedFrame*[nbKeyFrames];

  if (f.is_open())
  {
    for (i=0; i<nbKeyFrames; i++)     
    {
      getline (f,line);
      sscanf(line.c_str(), "%f %f %f %f %f %f",&index, &x, &y, &z, &vx, &ax);
      keyFrame_[i] = new qglviewer::ManipulatedFrame;
      keyFrame_[i]->setPosition((double)x, (double)y, 0.0);
      //keyFrame_[i]->setPosition(cos(i*2*M_PI/(nbKeyFrames-1)), sin(i*2*M_PI/(nbKeyFrames-1)), 0.0);
      kfi_.addKeyFrame(keyFrame_[i]);
    }
    f.close();
  }

  else cout << "Unable to open file"; 
  
  setCurrentKf(0);
  setManipulatedFrame(keyFrame_[currentKf()]);

  // Enable direct frame manipulation when the mouse hovers.
  setMouseTracking(false);

  setKeyDescription(Qt::Key_Plus, "Increases interpolation speed");
  setKeyDescription(Qt::Key_Minus, "Decreases interpolation speed");
  setKeyDescription(Qt::Key_Left, "Selects previous key frame");
  setKeyDescription(Qt::Key_Right, "Selects next key frame");
  setKeyDescription(Qt::Key_Return, "Starts/stops interpolation");

//   help();

  connect(&kfi_, SIGNAL(interpolated()), SLOT(updateGL()));
//   kfi_.startInterpolation();
}

void Curve::draw()
{
  // Draw interpolated frame
  glPushMatrix();
  glMultMatrixd(kfi_.frame()->matrix());
  drawAxis(0.01f);
  glPopMatrix();
  drawAxis(0.01f);
  glColor3f(_color_f1,_color_f2,_color_f3);
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
  glDisable(GL_LIGHTING);

  glLineWidth(2);
  glBegin(GL_LINES);
  for (unsigned int i=1; i< traj.size(); i++) {
       glVertex3f(traj[i-1].Pos[0], traj[i-1].Pos[1], traj[i-1].Pos[2]);
       glVertex3f(traj[i].Pos[0], traj[i].Pos[1], traj[i].Pos[2]);
  }
  glEnd();
  glPopAttrib();
 //kfi_.drawPath(1, 1);
  glColor3f(1.,1.,1.);
  return;
}

int Curve::nbKeyFrames()
{
 return _nbKeyFrames;
}  

int Curve::currentKf()
{
  return _currentKF; 
}

bool Curve::isDraw()
{
  return _isDraw;
}

void Curve::setIsDraw(bool v)
{
  _isDraw = v;
}

void Curve::setCurrentKf(int v)
{
 _currentKF = v;
}