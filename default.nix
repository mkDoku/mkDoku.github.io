{ mkDerivation, base, blaze-html, containers, hakyll, lib, pandoc
, pandoc-types, text
}:
mkDerivation {
  pname = "mkDokuBlog";
  version = "0.1.0.0";
  src = ./.;
  isLibrary = false;
  isExecutable = true;
  executableHaskellDepends = [
    base blaze-html containers hakyll pandoc pandoc-types text
  ];
  license = lib.licenses.bsd3;
  mainProgram = "site";
}
