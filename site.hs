--------------------------------------------------------------------------------
{-# LANGUAGE OverloadedStrings #-}
import           Data.Monoid                     (mappend)
import           Hakyll

-- for tags
import           Hakyll.Web.Tags                 ()
import           Text.Blaze.Html                 (toHtml, toValue, (!))
import           Text.Blaze.Html.Renderer.String (renderHtml)
import qualified Text.Blaze.Html5                as H
import qualified Text.Blaze.Html5.Attributes     as A

-- for LaTeX formula
-- see
-- http://travis.athougies.net/posts/2013-08-13-using-math-on-your-hakyll-blog.html
-- and
-- https://github.com/jdreaver/jdreaver.com/blob/master/hakyll.hs
--import qualified Data.Set            as S
import           Text.Pandoc.Options

-- For custom compiler with walkM transform
import           Data.Foldable                   (fold)
import           Text.Pandoc.Definition          (Block (..), Inline (..))
import           Text.Pandoc.Parsing             (extractReaderOptions)
import           Text.Pandoc.Walk                (walk, walkM)

import qualified Data.Text                       as T

--------------------------------------------------------------------------------
-- hakyll
--------------------------------------------------------------------------------
-- try to use
-- hakyllWith or hakyllWithArgs?
--
-- but what is the benefit?
--   * Use own config?
--   * Add Options?
--
main :: IO ()
main = hakyllWith defaultConfiguration myRules

--------------------------------------------------------------------------------
-- custom configuration
--------------------------------------------------------------------------------

test = 1

--------------------------------------------------------------------------------
-- custom rules
--------------------------------------------------------------------------------
-- splitte Rules so that each can be modified on its own
--
-- keep in mind that `Rules` are described by a "dynamic
-- scripting language" (DSL)
-- hence, the order of the `Rules` does not matter
myRules :: Rules ()
myRules =
     indexRules
  >> imagesRules
  >> fontsRules
  >> cssRules
  >> markdownRules
  >> postWithTagsRules
  >> blogRules
  >> tempalteRules


imagesRules :: Rules ()
imagesRules =
    match "images/**/*" $ do
        route idRoute
        compile copyFileCompiler

fontsRules :: Rules ()
fontsRules =
    match "fonts/**/*" $ do
        route idRoute
        compile copyFileCompiler

cssRules :: Rules ()
cssRules =
    match "css/*" $ do
      route   idRoute
      compile compressCssCompiler

markdownRules :: Rules ()
markdownRules =
  match (fromList ["about.markdown", "contact.markdown"]) $ do
      route   $ setExtension "html"
      -- compile $ pandocMathCompiler
      compile $ customCompiler
          >>= loadAndApplyTemplate "templates/default.html" defaultContext
          >>= relativizeUrls

postWithTagsRules :: Rules ()
postWithTagsRules = do
  -- taken from
  -- https://javran.github.io/posts/2014-03-01-add-tags-to-your-hakyll-blog.html
  --
  -- for single tag page see and try
  -- https://stackoverflow.com/questions/52805193/in-hakyll-how-can-i-generate-a-tags-page
  tags <- buildTags "posts/*" (fromCapture "tags/*.html")
  tagsRules tags $ \tag pat -> do
    let title = "Posts tagged \"" ++ tag ++ "\""
    route idRoute
    compile $ do
        posts <- recentFirst =<< loadAll pat
        let ctx = constField "title" title
                  `mappend` listField "posts" (postCtxWithTags tags) (return posts)
                  `mappend` defaultContext

        makeItem ""
            >>= loadAndApplyTemplate "templates/tag.html" ctx
            >>= loadAndApplyTemplate "templates/default.html" ctx
            >>= relativizeUrls

  match "posts/*" $ do
      -- Taken from https://javran.github.io/posts/2014-03-01-add-tags-to-your-hakyll-blog.html
      route $ setExtension "html"
      -- compile $ pandocMathCompiler
      compile $ customCompiler
          >>= loadAndApplyTemplate "templates/post.html"    (postCtxWithTags tags)
          >>= loadAndApplyTemplate "templates/default.html" (postCtxWithTags tags)
          >>= relativizeUrls

blogRules :: Rules ()
blogRules =
  create ["blog.html"] $ do
      route idRoute
      compile $ do
          posts <- recentFirst =<< loadAll "posts/*"
          let archiveCtx =
                  listField "posts" postCtx (return posts) `mappend`
                  constField "title" "Blog Posts"            `mappend`
                  defaultContext

          makeItem ""
              >>= loadAndApplyTemplate "templates/blog.html" archiveCtx
              >>= loadAndApplyTemplate "templates/default.html" archiveCtx
              >>= relativizeUrls

indexRules :: Rules ()
indexRules =
  match "index.html" $ do
      route idRoute
      compile $ do
          posts <- recentFirst =<< loadAll "posts/*"
          let indexCtx =
                  listField "posts" postCtx (return posts) `mappend`
                  defaultContext

          getResourceBody
              >>= applyAsTemplate indexCtx
              >>= loadAndApplyTemplate "templates/default.html" indexCtx
              >>= relativizeUrls

tempalteRules :: Rules ()
tempalteRules =
  match "templates/*" $ compile templateBodyCompiler


--------------------------------------------------------------------------------
-- custom context
--------------------------------------------------------------------------------
postCtx :: Context String
postCtx =
    dateField "date" "%B %e, %Y" `mappend`
    defaultContext

postCtxWithTags :: Tags -> Context String
postCtxWithTags tags = tagsField' "tags" tags `mappend` postCtx

tagsField' :: String -> Tags -> Context a
-- tagsFieldWith allows for using a custom tagsField
-- by providing
--   * getTags: how to get content (default)
--   * simpleRenderLink: how to render content (custom, see simpleRenderLink')
--
tagsField' = tagsFieldWith getTags simpleRenderLink' mconcat

simpleRenderLink' :: String -> Maybe FilePath -> Maybe H.Html
simpleRenderLink' _ Nothing = Nothing
simpleRenderLink' tag (Just filePath) =
  Just $
    -- use Text.Blaze.Html to generate custom element
    H.a
      ! A.href (toValue $ toUrl filePath)
      -- set class of element
      ! A.class_ "tag"
      $ toHtml tag

--------------------------------------------------------------------------------
-- custom compiler
--------------------------------------------------------------------------------
customCompiler :: Compiler (Item String)
customCompiler = pandocCompilerWithTransformM
               customHakyllReaderOptions
               customHakyllWriterOptions
               (walkM customTransform)

-- Adding 'synopsis' in code, so that
--
-- ```synopsis
-- some content
-- ```
--
-- can be used to style and modify 'some content'
--
-- inspired by
-- blog post: https://odone.io/posts/2020-06-08-custom-markdown-pandoc.html
-- code: https://github.com/3v0k4/contact-page/commit/539d89253879e903f3350e187885e4ac1f72a165
--
customTransform :: Block -> Compiler Block
customTransform ( CodeBlock (_, ["synopsis"], []) content) = do
      -- let wrapIt x = T.unpack x ++ "<br>"
      -- let content' = T.unlines $ map (T.pack . wrapIt) $ T.lines content
      pure
        $ RawBlock "html"
        $ fold
          [ "<details class=\"synopsis\">",
            "<summary class=\"synopsis\">",
            "TL;DR",
            "</summary>",
            "<span class=\"synopsis\">",
            content,
            "<br> Just give me the solution: <a href=\"#solution\">Solution</a>",
            "</span>",
            "</details>"
          ]
-- for everything else use the standard pandoc/hakyll compiler
customTransform x = pure x

customHakyllWriterOptions :: WriterOptions
customHakyllWriterOptions =
  let
    -- Added options for LaTeX formula using MathJax
    -- to make it work add
    --
    -- <script type="text/javascript"
    --    src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML">
    -- </script>
    --
    -- to templates/default.html after <!doctype html>
    --
    -- source:
    -- https://github.com/jdreaver/jdreaver.com/blob/master/hakyll.hs
    mathExtensions =
      [ Ext_tex_math_dollars
      , Ext_tex_math_double_backslash
      , Ext_latex_macros
      ]
    additionalExtensions =
      [ Ext_literate_haskell
      ]
    defaultExtensions = writerExtensions defaultHakyllWriterOptions
    newExtensions =
      foldr enableExtension defaultExtensions (mathExtensions ++ additionalExtensions)
    writerOptions =
      defaultHakyllWriterOptions
      { writerExtensions = newExtensions
      , writerHTMLMathMethod = MathJax ""
      }
  in writerOptions

customHakyllReaderOptions :: ReaderOptions
customHakyllReaderOptions = defaultHakyllReaderOptions

--------------------------------------------------------------------------------
-- custom templates
--------------------------------------------------------------------------------
