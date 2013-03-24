#if 0
// assimp 2
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#endif

// assimp 3
#include <assimp3/Importer.hpp>
#include <assimp3/Exporter.hpp>
#include <assimp3/postprocess.h>
#include <assimp3/scene.h>

// convex decomposition
#if COMPILE_CONVEX_DECOMPOSITION
#include "NvConvexDecomposition.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <cstdio>
#include <iostream>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#define iostream eus_iostream
#define complex  eus_complex

#include "eus.h"
extern "C" {
  pointer ___eus_assimp(register context *ctx, int n, pointer *argv, pointer env);
  void register_eus_assimp(){
    char modname[] = "___eus_assimp";
    return add_module_initializer(modname, (pointer (*)())___eus_assimp);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string
#undef iostream
#undef complex

#define SIZE_T_MAX (std::numeric_limits<size_t>::max())

static pointer K_VERTICES, K_NORMALS, K_INDICES, K_TYPE, K_MATERIAL;
static pointer K_LINES, K_TRIANGLES, K_QUADS, K_POLYGON, K_NAME;
static pointer K_AMBIENT, K_DIFFUSE, K_SPECULAR, K_EMISSION, K_SHININESS, K_TRANSPARENCY;

static std::string primitiveType (unsigned int tp) {
 switch (tp) {
 case aiPrimitiveType_POINT:
   return "POINT";
   break;
 case aiPrimitiveType_LINE:
   return "LINE";
   break;
 case aiPrimitiveType_TRIANGLE:
   return "TRIANGLE";
   break;
 case aiPrimitiveType_POLYGON:
   return "POLYGON";
   break;
 }
 return "NO_TYPE";
}

static void printMesh(aiMesh *mesh, std::string prefix)
{
  std::string name;
  name.assign(  mesh->mName.data, mesh->mName.length );

  std::cerr << ";;" << prefix << " Mesh name: " << name << std::endl;

  std::cerr << ";;" << prefix << "   Bones: " << mesh->mNumBones << std::endl;
  std::cerr << ";;" << prefix << "   Faces: " << mesh->mNumFaces << std::endl;
  std::cerr << ";;" << prefix << "   Vertices: " << mesh->mNumVertices << std::endl;
  std::cerr << ";;" << prefix << "   PType: " << primitiveType( mesh->mPrimitiveTypes ) << std::endl;
  std::cerr << ";;" << prefix << "   MaterialIndex: " << mesh->mMaterialIndex << std::endl;
}

static void printTransform(aiMatrix4x4 &transform, std::string prefix) {
  std::cerr << prefix << "  trans: " << transform.a1 << " " << transform.a2 << " " << transform.a3 << " " << transform.a4 << std::endl;
  std::cerr << prefix << "         " << transform.b1 << " " << transform.b2 << " " << transform.b3 << " " << transform.b4 << std::endl;
  std::cerr << prefix << "         " << transform.c1 << " " << transform.c2 << " " << transform.c3 << " " << transform.c4 << std::endl;
  std::cerr << prefix << "         " << transform.d1 << " " << transform.d2 << " " << transform.d3 << " " << transform.d4 << std::endl;
}

static void printNode(aiNode *node, std::string prefix)
{
  std::string name;
  name.assign (node->mName.C_Str());

  fprintf (stderr,"%s Node name(%lX): %s", prefix.c_str(), (void *)node, name.c_str());
  std::cerr << std::endl;
  std::cerr << prefix << "    Meshes: " << node->mNumMeshes << " / ";
  for (unsigned int i = 0; i < node->mNumMeshes; i++ ) {
    std::cerr << " " << node->mMeshes[i];
  }
  std::cerr << std::endl;
  printTransform(node->mTransformation, prefix);

  std::cerr << prefix << "   Children: " << node->mNumChildren << std::endl;
  for (unsigned int i = 0; i < node->mNumChildren; i++ ) {
    printNode(node->mChildren[i], prefix + "  ");
  }
}

static void printMaterial(aiMaterial *material, unsigned int idx)
{
  std::cerr << ";;  material: " << idx << std::endl;

  //float col[4];
  //aiReturn ret = material->Get(AI_MATKEY_COLOR_DIFFUSE, col, NULL);
  //std::cerr << "color : " << ret << " " << col[0] << " " << col[1] << " " << col[2] << " " << col[3];

  for ( unsigned int i = 0; i < material->mNumProperties; i++ ) {
    aiMaterialProperty *mp = material->mProperties[i];
    std::string kname;
    kname.assign(  mp->mKey.data, mp->mKey.length );
    std::cerr << ";;    " << kname << std::endl;
  }
}

static pointer store_mesh_info (register context *ctx, eusfloat_t base_scl,
                                const aiScene *scene, const aiNode *node, const aiMesh *amesh,
                                const aiMatrix4x4 &trans) {
  static int mesh_cntr = -1;
  pointer ver_mat, nom_mat, indices;
  eusfloat_t *ver_vec=NULL, *nom_vec=NULL;
  numunion nu;
  pointer ret = NIL;
  int npc = 0;

  mesh_cntr++;
  // name
  {
    std::string nname, mname;
    nname.assign (node->mName.C_Str());
    mname.assign (amesh->mName.C_Str());

    if (nname.empty() && mname.empty()) {
      nname = "eusglvertices_" + mesh_cntr;
    } else {
      nname = nname + "_" + mname;
    }
    pointer lname = makestring ((char *)nname.c_str(), nname.length());
    vpush (lname);
    lname = rawcons (ctx, lname , NIL);
    vpop (); vpush (lname);
    lname = rawcons (ctx, K_NAME, lname);
    vpop (); vpush (lname);
    ret = rawcons (ctx, lname, ret);
    vpop ();
    vpush (ret); npc++;
  }

  // type
  {
    pointer ltype = NIL;
    switch (amesh->mPrimitiveTypes) {
    case aiPrimitiveType_LINE:
      ltype = K_LINES;
      break;
    case aiPrimitiveType_TRIANGLE:
      ltype = K_TRIANGLES;
      break;
    case aiPrimitiveType_POLYGON:
      ltype = K_POLYGON;
      break;
    }
    ltype = rawcons (ctx, ltype , NIL);
    vpush (ltype);
    ltype = rawcons (ctx, K_TYPE, ltype);
    vpop(); vpush (ltype);
    ret = rawcons (ctx, ltype, ret);
    vpop();
    vpush (ret); npc++;
  }

  // material
  {
    pointer lmaterial = NIL;
    aiMaterial *am = scene->mMaterials[amesh->mMaterialIndex];
#if DEBUG
    std::cerr << ";; material properties: " << am->mNumProperties << std::endl;
#endif
    aiReturn ar;
    aiString s;
    aiColor4D clr4d( 0.0, 0.0, 0.0, 0.0);
    float val;
    int lpc = 0;
    ar = am->Get (AI_MATKEY_NAME, s);
    if (ar == aiReturn_SUCCESS) {
      std::string str; str.assign(s.C_Str());
#if DEBUG
      std::cerr << ";; material properties: name: " << str << std::endl;
#endif
      pointer pelem;
      pointer eusstr = makestring (s.data, strlen(s.data));
      vpush (eusstr);//
      pelem = rawcons (ctx, eusstr, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_NAME, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
    ar = am->Get (AI_MATKEY_COLOR_AMBIENT, clr4d);
    if (ar == aiReturn_SUCCESS) {
#if DEBUG
      std::cerr << ";; material properties: AMBIENT: ";
      std::cerr << clr4d[0] << " " << clr4d[1] << " " << clr4d[2] << " " << clr4d[3] << std::endl;
#endif
      pointer pelem = makefvector (4);
      pelem->c.fvec.fv[0] = clr4d[0]; pelem->c.fvec.fv[1] = clr4d[1];
      pelem->c.fvec.fv[2] = clr4d[2]; pelem->c.fvec.fv[3] = clr4d[3];
      vpush (pelem);//
      pelem = rawcons (ctx, pelem, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_AMBIENT, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
    ar = am->Get (AI_MATKEY_COLOR_DIFFUSE, clr4d);
    if (ar == aiReturn_SUCCESS) {
#if DEBUG
      std::cerr << ";; material properties: DIFFUSE: ";
      std::cerr << clr4d[0] << " " << clr4d[1] << " " << clr4d[2] << " " << clr4d[3] << std::endl;
#endif
      pointer pelem = makefvector (4);
      pelem->c.fvec.fv[0] = clr4d[0]; pelem->c.fvec.fv[1] = clr4d[1];
      pelem->c.fvec.fv[2] = clr4d[2]; pelem->c.fvec.fv[3] = clr4d[3];
      vpush (pelem);//
      pelem = rawcons (ctx, pelem, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_DIFFUSE, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
    ar = am->Get (AI_MATKEY_COLOR_SPECULAR, clr4d);
    if (ar == aiReturn_SUCCESS) {
#if DEBUG
      std::cerr << ";; material properties: SPECULAR: ";
      std::cerr << clr4d[0] << " " << clr4d[1] << " " << clr4d[2] << " " << clr4d[3] << std::endl;
#endif
      pointer pelem = makefvector (4);
      pelem->c.fvec.fv[0] = clr4d[0]; pelem->c.fvec.fv[1] = clr4d[1];
      pelem->c.fvec.fv[2] = clr4d[2]; pelem->c.fvec.fv[3] = clr4d[3];
      vpush (pelem);//
      pelem = rawcons (ctx, pelem, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_SPECULAR, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
    ar = am->Get (AI_MATKEY_COLOR_EMISSIVE, clr4d);
    if (ar == aiReturn_SUCCESS) {
#if DEBUG
      std::cerr << ";; material properties: EMISSIVE: ";
      std::cerr << clr4d[0] << " " << clr4d[1] << " " << clr4d[2] << " " << clr4d[3] << std::endl;
#endif
      pointer pelem = makefvector (4);
      pelem->c.fvec.fv[0] = clr4d[0]; pelem->c.fvec.fv[1] = clr4d[1];
      pelem->c.fvec.fv[2] = clr4d[2]; pelem->c.fvec.fv[3] = clr4d[3];
      vpush (pelem);//
      pelem = rawcons (ctx, pelem, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_EMISSION, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
    ar = am->Get (AI_MATKEY_SHININESS, val);
    if (ar == aiReturn_SUCCESS) {
#if DEBUG
      std::cerr << ";; material properties: SHININESS: " << val << std::endl;
#endif
      pointer pelem;
      pelem = rawcons (ctx, makeflt(val), NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_SHININESS, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
#if 0 // Do not use transparent??
    ar = am->Get (AI_MATKEY_COLOR_TRANSPARENT, clr4d);
    if (ar == aiReturn_SUCCESS) {
      std::cerr << ";; material properties: TRANSPARENT: ";
      std::cerr << clr4d[0] << " " << clr4d[1] << " " << clr4d[2] << " " << clr4d[3] << std::endl;
      pointer pelem = makefvector (4);
      pelem->c.fvec.fv[0] = clr4d[0]; pelem->c.fvec.fv[1] = clr4d[1];
      pelem->c.fvec.fv[2] = clr4d[2]; pelem->c.fvec.fv[3] = clr4d[3];
      vpush (pelem);//
      pelem = rawcons (ctx, pelem, NIL);
      vpush (pelem);//
      pelem = rawcons (ctx, K_TRANSPARENT, pelem);
      vpush (pelem);//
      lmaterial = rawcons (ctx, pelem, lmaterial);
      vpop(); vpop(); vpop();
      vpush (lmaterial); lpc++;
    }
#endif
    pointer tmp = rawcons (ctx, lmaterial, NIL);
    vpush (tmp); lpc++;
    tmp = rawcons (ctx, K_MATERIAL, tmp);
    vpush (tmp); lpc++;
    ret = rawcons (ctx, tmp, ret);
    while(lpc-- > 0) vpop();

    vpush (ret); npc++;
  }

  if (!!amesh->mNormals) {
    nom_mat = makematrix (ctx, amesh->mNumVertices, 3); vpush (nom_mat); npc++;
    nom_vec = nom_mat->c.ary.entity->c.fvec.fv;
  }
  ver_mat = makematrix (ctx, amesh->mNumVertices, 3); vpush (ver_mat); npc++;
  ver_vec = ver_mat->c.ary.entity->c.fvec.fv;

  // count indices
  {
    int icount = 0;
    for (unsigned int f = 0; f < amesh->mNumFaces; f++) {
      icount += amesh->mFaces[f].mNumIndices;
    }
    indices = makevector (C_INTVECTOR, icount);
    vpush (indices); npc++;
  }
  // fill indices
  {
    eusinteger_t *vec = indices->c.ivec.iv;
    int icount = 0;
    for (unsigned int f = 0; f < amesh->mNumFaces; f++) {
      for(unsigned int p = 0; p < amesh->mFaces[f].mNumIndices; p++) {
        vec[icount++] = amesh->mFaces[f].mIndices[p];
      }
    }
  }
  // add indices to return list
  {
    pointer tmp;
    tmp = rawcons (ctx, indices, NIL);
    vpush (tmp);
    tmp = rawcons (ctx, K_INDICES, tmp);
    ret = rawcons (ctx, tmp, ret);
    vpop();
    vpush (ret); npc++;
  }
  // make vetices and normal matrix
  int vcount=0, ncount=0;
  for (unsigned int i = 0; i < amesh->mNumVertices; ++i) {
    aiQuaternion rot;
    aiVector3D   pos;
    trans.DecomposeNoScaling(rot, pos);
    aiVector3D tvec (amesh->mVertices[i]);
    tvec *= trans;
    tvec *= base_scl;
    ver_vec[vcount++] = tvec.x;
    ver_vec[vcount++] = tvec.y;
    ver_vec[vcount++] = tvec.z;
    if ( !!nom_vec ) {
      aiVector3D tnom = rot.Rotate (amesh->mNormals[i]);
      nom_vec[ncount++] = tnom.x;
      nom_vec[ncount++] = tnom.y;
      nom_vec[ncount++] = tnom.z;
    }
  }
  //
  if ( !!nom_vec ) {
    pointer tmp;
    tmp = rawcons (ctx, nom_mat , NIL);
    vpush (tmp);
    tmp = rawcons (ctx, K_NORMALS , tmp);
    ret = rawcons (ctx, tmp, ret);
    vpop();
    vpush (ret); npc++;
  }
  //
  {
    pointer tmp;
    tmp = rawcons (ctx, ver_mat , NIL);
    vpush (tmp);
    tmp = rawcons (ctx, K_VERTICES , tmp);
    ret = rawcons (ctx, tmp, ret);
    vpop();
    vpush (ret); npc++;
  }

  for (;npc > 0; npc--) vpop();

  return ret;
}

static void register_all_nodes (register context *ctx, eusfloat_t base_scl,
                                const aiScene *scene, const aiNode *node,
                                const aiMatrix4x4 &parent_world_trans,
                                std::vector<pointer> &mesh_info) {
  aiMatrix4x4 node_world_trans = parent_world_trans * node->mTransformation;
#if DEBUG
  fprintf (stderr, "node : %lX\n", (void *)node);
  printTransform (node_world_trans, "");
#endif
  if (node->mNumMeshes > 0 && node->mMeshes != NULL) {
    for (unsigned int n = 0; n < node->mNumMeshes; n++) {
#if DEBUG
      fprintf (stderr, " mesh: %d", node->mMeshes[n]);
#endif
      aiMesh *am = scene->mMeshes[node->mMeshes[n]];
      std::cerr << ";; mesh = " << (void *)am << std::endl;
      pointer ret = store_mesh_info (ctx, base_scl, scene, node, am, node_world_trans);
      vpush (ret);
      mesh_info.push_back (ret);
    }
  }
#if DEBUG
  fprintf(stderr, "\n");
  std::cerr << " children: " <<  node->mNumChildren << std::endl;
#endif
  for (unsigned int c = 0; c < node->mNumChildren; c++) {
    register_all_nodes (ctx, base_scl,
                        scene, node->mChildren[c],
                        node_world_trans, mesh_info);
  }
}

pointer GET_MESHES(register context *ctx,int n,pointer *argv)
{
  /* filename &optional scale (gen_normal nil) (smooth_normal nil) (split_large_mesh nil) (optimize_mesh nil) (identical_vert nil) (fix_normal nil) (dumpfilename) */
  eusfloat_t base_scl = 1.0;
  numunion nu;

  ckarg2(1, 9);
  if (!isstring(argv[0])) error (E_NOSTRING);
  if (n > 1) {
    base_scl = ckfltval (argv[1]);
  }

  Assimp::Importer importer;
  unsigned int post_proc = (aiProcess_Triangulate | aiProcess_SortByPType);
  bool recalc_normal = false;
  char *dumpfile = NULL;

  if (n > 2) {
    if (argv[2] != NIL) {
      post_proc |= (aiProcess_GenNormals | aiProcess_FindInvalidData);
      recalc_normal = true;
    }
  }
  if (n > 3) {
    if (argv[3] != NIL) {
      post_proc |= (aiProcess_GenSmoothNormals | aiProcess_FindInvalidData);
      recalc_normal = true;
    }
  }
  if (n > 4) {
    if (argv[4] != NIL) { post_proc |= aiProcess_SplitLargeMeshes; }
  }
  if (n > 5) {
    if (argv[5] != NIL) { post_proc |= aiProcess_OptimizeMeshes; }
  }
  if (n > 6) {
    if (argv[6] != NIL) { post_proc |= aiProcess_JoinIdenticalVertices; }
  }
  if (n > 7) {
    if (argv[7] != NIL) { post_proc |= aiProcess_FixInfacingNormals; }
  }
  if (n > 8) {
    if (argv[8] != NIL) {
      if (!isstring(argv[8])) error(E_NOSTRING);
      dumpfile = (char *)get_string(argv[8]);
    }
  }
  const aiScene* raw_scene = importer.ReadFile ((char *)get_string(argv[0]), 0);

  if (!raw_scene) {
    std::string str (importer.GetErrorString());
    std::cerr << ";; " << str << std::endl;
    return NIL;
  }
#if DEBUG
  std::cerr << ";; Num meshes(raw): " << raw_scene->mNumMeshes << std::endl;
#endif
  if (recalc_normal) {
#if DEBUG
    std::cerr << ";; Recalc_normal" << std::endl;
#endif
    for (unsigned int m = 0; m < raw_scene->mNumMeshes; m++) {
      aiMesh *a = raw_scene->mMeshes[m];
      if (!!a->mNormals) { a->mNormals = NULL; }
    }
  }

  const aiScene* scene = importer.ApplyPostProcessing (post_proc);
  if (!scene) {
    std::string str (importer.GetErrorString());
    std::cerr << ";; " << str << std::endl;
    return NIL;
  }
#if DEBUG
  std::cerr << ";; Num meshes: " << scene->mNumMeshes << std::endl;
  // travarse nodes and store
  printNode (scene->mRootNode, ";;");
#endif

  pointer lst = NIL;
  {
    aiMatrix4x4 world_trans;
    std::vector<pointer> mesh_info;
    register_all_nodes (ctx, base_scl, scene,
                        scene->mRootNode, world_trans, mesh_info);
#if DEBUG
    std::cerr << ";; mesh size: " << mesh_info.size() << std::endl;
#endif
    for (std::vector<pointer>::reverse_iterator rit = mesh_info.rbegin();
         rit != mesh_info.rend(); rit++) {
      vpush(lst);
      lst = rawcons (ctx, *rit, lst);
      vpop(); // vpop(); // pop for mesh_info
    }
    vpush (lst);
  }

  if (!!dumpfile) {
    std::string outf, outext;
    outf.assign (dumpfile);
    {
      const std::string::size_type s = outf.find_last_of('.');
      if (s != std::string::npos) {
        outext = outf.substr (s+1);
      } else {
        std::cerr << ";; Can not find extention: " << outf << std::endl;
      }
    }
    Assimp::Exporter exporter;
    size_t outfi = SIZE_T_MAX;
    for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
      const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
      if (outext == e->fileExtension) {
        outfi = i; break;
      }
    }
    if (outfi == SIZE_T_MAX) {
      outext = "stl";
      for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
        const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
        if (outext == e->fileExtension) {
          outfi = i; break;
        }
      }
    }
    if (outfi == SIZE_T_MAX) outfi = 0;

    const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(outfi);
#if DEBUG
    fprintf (stderr, ";; use file format: %s\n", e->id);
#endif
    aiReturn ret = exporter.Export (scene, e->id, outf);
  }

  vpop(); // vpush (lst);
  return lst;
}

pointer DUMP_GL_VERTICES(register context *ctx,int n,pointer *argv)
{
  /* filename type-list material-list vertices-list normals-list indices-list
     (scale) (gen_normal nil) (smooth_normal nil) (split_large_mesh nil)
     (optimize_mesh nil) (identical_vert nil) (fix_normal)
  */
  // FIXME: name_list
  ckarg2(6, 12);
  numunion nu;

  char *dumpfile = NULL;
  if (!isstring(argv[0])) error(E_NOSTRING);
  dumpfile = (char *)get_string(argv[0]);

  int list_len = 0;
  {
    pointer a = argv[1];
    while (islist(a)) {list_len++; a=ccdr(a);}
  }
  if (list_len <= 0) return NIL;
  bool has_materials = false;
  pointer ltype, linfo, lvertices, lnormals, lindices;
  pointer mtype, minfo, mvertices, mnormals, mindices;
  ltype = argv[1];
  linfo = argv[2];
  lvertices = argv[3];
  lnormals = argv[4];
  lindices = argv[5];

  // assimp loader
  aiScene *pScene = (aiScene *) malloc(sizeof (aiScene));
  memset((void *)pScene, 0, sizeof (aiScene));
  pScene->mPrivate = malloc(sizeof (void *) * 2);
  //printf("scene = %lX\n", (void *)pScene);

  pScene->mNumMeshes = list_len;
  pScene->mMeshes = new aiMesh*[list_len];

  // allocate a single node
  pScene->mRootNode = new aiNode();
  pScene->mRootNode->mNumMeshes = list_len;
  pScene->mRootNode->mMeshes = new unsigned int[list_len];
  pScene->mRootNode->mName.Set("eus_glvertices");

  if (linfo != NIL) {
    pScene->mNumMaterials = list_len;
    pScene->mMaterials = new aiMaterial*[list_len];
    has_materials = true;
  } else {
    pScene->mNumMaterials = 1;
    pScene->mMaterials = new aiMaterial*[1];
    has_materials = false;
    // make default material
    aiMaterial* pcMat = new aiMaterial();
    aiString s; s.Set (AI_DEFAULT_MATERIAL_NAME);
    pcMat->AddProperty (&s, AI_MATKEY_NAME);
    aiColor4D clrDiffuse(0.8f, 0.8f, 0.8f, 1.0f);
    pcMat->AddProperty (&clrDiffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
    pcMat->AddProperty (&clrDiffuse, 1, AI_MATKEY_COLOR_SPECULAR);
    clrDiffuse = aiColor4D (0.2f, 0.2f, 0.2f, 1.0f);
    pcMat->AddProperty (&clrDiffuse, 1, AI_MATKEY_COLOR_AMBIENT);

    pScene->mMaterials[0] = pcMat;
  }

  for (int i = 0; i < list_len; i++) {
    mtype = ccar (ltype);
    minfo = ccar (linfo);
    mvertices = ccar (lvertices);
    mnormals = ccar (lnormals);
    mindices = ccar (lindices);

    ltype = ccdr (ltype);
    linfo = ccdr (linfo);
    lvertices = ccdr (lvertices);
    lnormals = ccdr (lnormals);
    lindices = ccdr (lindices);

    pScene->mRootNode->mMeshes[i] = i;
    aiMesh* pMesh = pScene->mMeshes[i] = new aiMesh();
    std::cerr << ";; mesh = " << (void *)pMesh << std::endl;
    if (!has_materials) {
      pMesh->mMaterialIndex = 0;
    } else {
      pMesh->mMaterialIndex = i;
      // material
      aiMaterial* pcMat = new aiMaterial();
      std::ostringstream oss;
      oss << "eusassimp_mat_" << i;
      aiString s; s.Set (oss.str());
      pcMat->AddProperty(&s, AI_MATKEY_NAME);

      for (int el = 0; el < 6; el++) {
        // (list :ambient :diffuse :specular :emission :shininess :transparency)
        pointer melem = ccar (minfo);
        minfo = ccdr (minfo);
        aiColor4D clr4d (0.0f, 0.0f, 0.0f, 1.0f);
        switch (el) {
        case 0:
          if (melem != NIL) {
            for(int j = 0; j < intval(melem->c.fvec.length); j++) {
              clr4d[j] = melem->c.fvec.fv[j];
            }
            //printf("%d: %f %f %f %f\n", el, clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
            pcMat->AddProperty(&clr4d, 1, AI_MATKEY_COLOR_AMBIENT);
          }
          break;
        case 1:
          if (melem != NIL) {
            for(int j = 0; j < intval(melem->c.fvec.length); j++) {
              clr4d[j] = melem->c.fvec.fv[j];
            }
            //printf("%d: %f %f %f %f\n", el, clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
            pcMat->AddProperty(&clr4d, 1, AI_MATKEY_COLOR_DIFFUSE);
          }
          break;
        case 2:
          if (melem != NIL) {
            for(int j = 0; j < intval(melem->c.fvec.length); j++) {
              clr4d[j] = melem->c.fvec.fv[j];
            }
            //printf("%d: %f %f %f %f\n", el, clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
            pcMat->AddProperty(&clr4d, 1, AI_MATKEY_COLOR_SPECULAR);
          }
          break;
        case 3:
          if (melem != NIL) {
            for(int j = 0; j < intval(melem->c.fvec.length); j++) {
              clr4d[j] = melem->c.fvec.fv[j];
            }
            //printf("%d: %f %f %f %f\n", el, clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
            pcMat->AddProperty(&clr4d, 1, AI_MATKEY_COLOR_EMISSIVE);
          }
          break;
        case 4:
          if (melem != NIL) {
            float val = fltval (melem);
            pcMat->AddProperty(&val, 1, AI_MATKEY_SHININESS);
          }
          break;
        case 5:
          if (melem != NIL) {
            if (isnum (melem)) {
              float val = isflt (melem) ? fltval (melem) : (float)intval (melem);
              //printf("trans: %f\n", val);
              for(int j = 0; j < 4; j++) {
                clr4d[j] = val;
              }
            } else {
              for(int j = 0; j < intval(melem->c.fvec.length); j++) {
                clr4d[j] = melem->c.fvec.fv[j];
              }
            }
            //printf("%d: %f %f %f %f\n", el, clr4d[0], clr4d[1], clr4d[2], clr4d[3]);
            pcMat->AddProperty(&clr4d, 1, AI_MATKEY_COLOR_TRANSPARENT);
          }
          break;
        }
      }
      pScene->mMaterials[i] = pcMat;
    }
    // vertices
    if (mvertices == NIL) { /* error */ }
    eusinteger_t size = intval (mvertices->c.ary.entity->c.fvec.length);
    eusfloat_t *fv = mvertices->c.ary.entity->c.fvec.fv;
    pMesh->mNumVertices = size / 3;
    pMesh->mVertices = new aiVector3D [pMesh->mNumVertices];
    for (unsigned int k = 0; k < pMesh->mNumVertices; k++) {
      pMesh->mVertices[k].x = *fv++;
      pMesh->mVertices[k].y = *fv++;
      pMesh->mVertices[k].z = *fv++;
    }
    // normals
    if (mnormals != NIL) {
      eusinteger_t size = intval (mnormals->c.ary.entity->c.fvec.length);
      eusfloat_t *fv = mnormals->c.ary.entity->c.fvec.fv;
      if (size / 3 != pMesh->mNumVertices) { /* error */ }
      pMesh->mNormals  = new aiVector3D [pMesh->mNumVertices];
      for (unsigned int k = 0; k < pMesh->mNumVertices; k++) {
        pMesh->mNormals[k].x = *fv++;
        pMesh->mNormals[k].y = *fv++;
        pMesh->mNormals[k].z = *fv++;
      }
    }
    unsigned int verts_per_face = intval(mtype);
    if (mindices != NIL) {
      if (verts_per_face != 4 && verts_per_face != 3) {
        /* variable face size */
        // not implemented yet
      } else {
        eusinteger_t *iv = mindices->c.ivec.iv;
        eusinteger_t idlen = intval (mindices->c.ivec.length);
        pMesh->mNumFaces = idlen / verts_per_face;//

        pMesh->mFaces = new aiFace[pMesh->mNumFaces];
        for (unsigned int f = 0; f < pMesh->mNumFaces; f++) {
          aiFace& face = pMesh->mFaces[f];
          face.mNumIndices = verts_per_face;
          face.mIndices = new unsigned int[verts_per_face];
          for (unsigned int o = 0; o < verts_per_face; o++) {
            face.mIndices[o] = *iv++;
          }
        }
      }
    } else {
      // add indices
      if (verts_per_face != 4 && verts_per_face != 3) { /* error */ }
      pMesh->mNumFaces = pMesh->mNumVertices / verts_per_face;
      pMesh->mFaces = new aiFace[pMesh->mNumFaces];
      for (unsigned int f = 0, p = 0; f < pMesh->mNumFaces; f++) {
        aiFace& face = pMesh->mFaces[f];
        face.mNumIndices = verts_per_face;
        face.mIndices = new unsigned int[verts_per_face];
        for (unsigned int o = 0; o < verts_per_face; o++, p++) {
          face.mIndices[o] = p;
        }
      }
    }
  }

  if (!!dumpfile) {
    std::string outf, outext;
    outf.assign (dumpfile);
    {
      const std::string::size_type s = outf.find_last_of('.');
      if (s != std::string::npos) {
        outext = outf.substr (s+1);
      } else {
        std::cerr << ";; Can not find extention: " << outf << std::endl;
      }
    }

    Assimp::Exporter exporter;
    size_t outfi = SIZE_T_MAX;
    for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
      const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
      if (outext == e->fileExtension) {
        outfi = i; break;
      }
    }
    if (outfi == SIZE_T_MAX) {
      outext = "stl";
      for(size_t i = 0, end = exporter.GetExportFormatCount(); i < end; ++i) {
        const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
        if (outext == e->fileExtension) {
          outfi = i; break;
        }
      }
    }
    if (outfi == SIZE_T_MAX) outfi = 0;
    const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(outfi);
    aiReturn ret = exporter.Export (pScene, e->id, outf);
  }

  return NIL;
}

pointer CONVEX_DECOMP_GL_VERTICES(register context *ctx,int n,pointer *argv)
{
  /* vertices-list indices-list (params) */
  numunion nu;
  int verts_per_face = 3;
  pointer lvertices = argv[0];
  pointer lindices = argv[1];
  pointer ret = NIL;

#if COMPILE_CONVEX_DECOMPOSITION
  int list_len = 0;
  {
    pointer a = lvertices;
    while (islist (a)) {list_len++; a=ccdr(a);}
  }
  //
  CONVEX_DECOMPOSITION::iConvexDecomposition *ic = CONVEX_DECOMPOSITION::createConvexDecomposition();

  // store all vertices
  for (int i = 0; i < list_len; i++) {
    pointer mvertices = ccar (lvertices);
    pointer mindices = ccar (lindices);

    lvertices = ccdr (lvertices);
    lindices = ccdr (lindices);

    if (mvertices == NIL) { /* error */ }
    eusinteger_t size = intval (mvertices->c.ary.entity->c.fvec.length);
    eusfloat_t *fv = mvertices->c.ary.entity->c.fvec.fv;

    if (mindices != NIL) {
      eusinteger_t *iv = mindices->c.ivec.iv;
      eusinteger_t idlen = intval (mindices->c.ivec.length);
      for (unsigned int f = 0; f < idlen / verts_per_face; f++) {
        eusinteger_t i1 = *iv++;
        eusinteger_t i2 = *iv++;
        eusinteger_t i3 = *iv++;
        NxF32 p1[3];
        NxF32 p2[3];
        NxF32 p3[3];

        p1[0] = fv[3*i1];
        p1[1] = fv[3*i1+1];
        p1[2] = fv[3*i1+2];

        p2[0] = fv[3*i2];
        p2[1] = fv[3*i2+1];
        p2[2] = fv[3*i2+2];

        p3[0] = fv[3*i3];
        p3[1] = fv[3*i3+1];
        p3[2] = fv[3*i3+2];

        ic->addTriangle (p1, p2, p3);
      }
    } else {
      int step = (3 * verts_per_face);
      for (unsigned int f = 0; f < size / step; f++) {
        NxF32 p1[3];
        NxF32 p2[3];
        NxF32 p3[3];

        p1[0] = fv[f * step + 0];
        p1[1] = fv[f * step + 1];
        p1[2] = fv[f * step + 2];

        p2[0] = fv[f * step + 3];
        p2[1] = fv[f * step + 4];
        p2[2] = fv[f * step + 5];

        p3[0] = fv[f * step + 6];
        p3[1] = fv[f * step + 7];
        p3[2] = fv[f * step + 8];

        ic->addTriangle (p1, p2, p3);
      }
    }
  }

  NxF32 skinWidth = 0;
  NxU32 decompositionDepth = 4;
  NxU32 maxHullVertices    = 64;
  NxF32 concavityThresholdPercent = 0.1f;
  NxF32 mergeThresholdPercent = 20;
  NxF32 volumeSplitThresholdPercent = 2;
  bool useInitialIslandGeneration = true;
  bool useIslandGeneration = false;
  bool useBackgroundThreads = false;
  ic->computeConvexDecomposition (skinWidth,
                                  decompositionDepth,
                                  maxHullVertices,
                                  concavityThresholdPercent,
                                  mergeThresholdPercent,
                                  volumeSplitThresholdPercent,
                                  useInitialIslandGeneration,
                                  useIslandGeneration,
                                  useBackgroundThreads);

  while ( !ic->isComputeComplete() ) {
#if DEBUG
    fprintf (stderr, "Computing the convex decomposition in a background thread.\r\n");
#endif
    sleep(1);
    // Sleep(1000);
  }
  // finish process

  NxU32 hullCount = ic->getHullCount();
#if DEBUG
  fprintf (stderr, "Convex Decomposition produced %d hulls.\r\n", hullCount );
#endif
  for (NxU32 i = 0; i < hullCount; i++) {
    //
    pointer tmpret = NIL;
    int npc = 0;
    // name
    {
      std::ostringstream oss;
      oss << "convex_" << i;
      pointer lname = makestring ((char *)oss.str().c_str(), oss.str().length());
      vpush (lname);
      lname = rawcons (ctx, lname , NIL);
      vpop (); vpush (lname);
      lname = rawcons (ctx, K_NAME, lname);
      vpop (); vpush (lname);
      tmpret = rawcons (ctx, lname, tmpret);
      vpop ();
      vpush (tmpret); npc++;
    }
    // type
    {
      pointer ltype = K_TRIANGLES;
      ltype = rawcons (ctx, ltype , NIL);
      vpush (ltype);
      ltype = rawcons (ctx, K_TYPE, ltype);
      vpop(); vpush (ltype);
      tmpret = rawcons (ctx, ltype, tmpret);
      vpop();
      vpush (tmpret); npc++;
    }

    CONVEX_DECOMPOSITION::ConvexHullResult result;
    ic->getConvexHullResult(i, result);

    pointer ver_mat = makematrix (ctx, result.mVcount, 3); vpush (ver_mat); npc++;
    eusfloat_t *ver_vec = ver_mat->c.ary.entity->c.fvec.fv;

    pointer indices = makevector (C_INTVECTOR, result.mTcount * 3);
    eusinteger_t *vec = indices->c.ivec.iv;
    vpush (indices); npc++;

    for (NxU32 i = 0; i < result.mVcount * 3; i++) {
      ver_vec[i] = result.mVertices[i];
    }
    for (NxU32 i = 0; i < result.mTcount * 3; i++) {
      vec[i] = result.mIndices[i];
    }
    // add indices to return list
    {
      pointer tmp;
      tmp = rawcons (ctx, indices, NIL);
      vpush (tmp);
      tmp = rawcons (ctx, K_INDICES, tmp);
      tmpret = rawcons (ctx, tmp, tmpret);
      vpop();
      vpush (tmpret); npc++;
    }
    //
    {
      pointer tmp;
      tmp = rawcons (ctx, ver_mat , NIL);
      vpush (tmp);
      tmp = rawcons (ctx, K_VERTICES , tmp);
      tmpret = rawcons (ctx, tmp, tmpret);
      vpop();
      vpush (tmpret); npc++;
    }

    vpush(ret); npc++;
    ret = rawcons (ctx, tmpret, ret);
    for (;npc > 0; npc--) vpop();
    vpush(ret);
  }

  CONVEX_DECOMPOSITION::releaseConvexDecomposition(ic);
  vpop(); // pop ret
#endif

  return ret;
}

pointer ASSIMP_DESCRIBE(register context *ctx,int n,pointer *argv)
{
  /* */
  ckarg(1);
  if (!isstring(argv[0])) error(E_NOSTRING);

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile((char *)get_string(argv[0]),
                                           aiProcess_Triangulate            |
                                           aiProcess_JoinIdenticalVertices  |
                                           aiProcess_SortByPType);

  if (!scene) {
    std::string str( importer.GetErrorString() );
    std::cerr << ";; " << str << std::endl;
    //std::cerr << ";; file: " << get_string(argv[0]) << " not found" << std::endl;
    return NIL;
  }

  std::cerr << ";; scene->NumAnimations " <<  scene->mNumAnimations << std::endl;
  //aiAnimation ** mAnimations;

  std::cerr << ";; scene->mNumCameras " << scene->mNumCameras << std::endl;
  //aiCamera ** mCameras;

  std::cerr << ";; scene->mNumLights " << scene->mNumLights << std::endl;
  //aiLight ** mLights;

  std::cerr << ";; scene->mNumMaterials " << scene->mNumMaterials << std::endl;
  for (unsigned int i = 0; i < scene->mNumMaterials; i++) {
    printMaterial(scene->mMaterials[i], i);
  }
  //aiMaterial ** mMaterials;

  std::cerr << ";; scene->mNumTextures " << scene->mNumTextures << std::endl;
  //aiTexture ** mTextures;

  std::cerr << ";; scene->mFlags " << scene->mFlags << std::endl;

  std::cerr << ";; scene->mNumMeshes " << scene->mNumMeshes << std::endl;

  //aiMesh ** scene->mMeshes;
  for ( unsigned int i = 0; i < scene->mNumMeshes; i++ ) {
    printMesh( scene->mMeshes[i] , "");
  }

  printNode( scene->mRootNode , "");

  return NIL;
}

pointer ___eus_assimp(register context *ctx, int n, pointer *argv, pointer env)
{
  defun(ctx,"ASSIMP-GET-GLVERTICES", argv[0], (pointer (*)())GET_MESHES);
  defun(ctx,"ASSIMP-DUMP-GLVERTICES", argv[0], (pointer (*)())DUMP_GL_VERTICES);
  defun(ctx,"C-CONVEX-DECOMPOSITION-GLVERTICES", argv[0], (pointer (*)())CONVEX_DECOMP_GL_VERTICES);
  defun(ctx,"ASSIMP-DESCRIBE", argv[0], (pointer (*)())ASSIMP_DESCRIBE);

  K_VERTICES  = defkeyword(ctx, "VERTICES");
  K_NORMALS   = defkeyword(ctx, "NORMALS");
  K_INDICES   = defkeyword(ctx, "INDICES");
  K_MATERIAL  = defkeyword(ctx, "MATERIAL");
  K_TYPE      = defkeyword(ctx, "TYPE");
  K_LINES     = defkeyword(ctx, "LINES");
  K_TRIANGLES = defkeyword(ctx, "TRIANGLES");
  K_QUADS     = defkeyword(ctx, "QUADS");
  K_POLYGON   = defkeyword(ctx, "POLYGON");
  K_NAME      = defkeyword(ctx, "NAME");
  // for material
  K_AMBIENT      = defkeyword(ctx, "AMBIENT");
  K_DIFFUSE      = defkeyword(ctx, "DIFFUSE");
  K_SPECULAR     = defkeyword(ctx, "SPECULAR");
  K_EMISSION     = defkeyword(ctx, "EMISSION");
  K_SHININESS    = defkeyword(ctx, "SHININESS");
  K_TRANSPARENCY = defkeyword(ctx, "TRANSPARENCY");

  return 0;
}
